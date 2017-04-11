%%%%%%%%%%%%% CALCULOS DEL TP DE MATLAB Y SIMULINK DE CONTROL

%%%% Parametros del motor
Ra = 2.67;
La = 23e-3;
Kt = 12e-3;
Kb = 12e-3;
Jm = 31.5e-6;
Bm = 63e-6;

%%%% Parametros del resto del circuito
Kpot1 = 5.8;
Kpot2 = 5.8;
Ktac = 22e-3;
Caja_reductora = 30;
Amplificador = 115;
integrador = [ 1 0 ];


% Demas cosas generales
fecha_ejecucion = clock;     % Se usa la fecha de ejecucion del script como nombre para la carpeta donde van las imagenes de salida
carpeta_salida = [ 'Imagenes/1c_' num2str( fecha_ejecucion(1) ) '_' num2str( fecha_ejecucion(2) ) '_' num2str( fecha_ejecucion(3) ) '_' num2str( fecha_ejecucion(4) ) '_' num2str( fecha_ejecucion(5) ) ];
mkdir( carpeta_salida );



%%%%%% EJERCICIO 1C: Agregado de un rozamiento extra de B = 10Bm


    %%%% MODELADO DEL MOTOR
    Polo_electrico = [ La Ra ];
    Polo_mecanico = [ Jm 10*Bm ];
    G_motor_denominador = conv( Polo_electrico, Polo_mecanico );
    G_motor_numerador = Kt;
    G_motor = tf( G_motor_numerador, G_motor_denominador );
    H_motor_numerador = Kb;
    H_motor_denominador = 1;
    H_motor = tf( H_motor_numerador, H_motor_denominador );
    M_motor = feedback( G_motor, H_motor );
    G_reductor = tf( 1, [ Caja_reductora 0 ] );
    M_motor_reductor = series( M_motor, G_reductor );
    G_total = M_motor_reductor * Kpot1 * Amplificador;
    H_total = tf( 1 );
    GH_total_lazo_abierto = series( G_total, H_total );

    % Trazado del lugar de raices
    figure();
    rlocus( GH_total_lazo_abierto );
    title( 'Lugar de raices de GH a lazo abierto con B = 10Bm. Parametro variable: K' );
    print( [ carpeta_salida '/Lugar_Raices_Completo_10Bm' '.jpg' ], '-djpeg90' );
    
    % Valores obtenidos del lugar de raices
    K_sobre_amortiguamiento = 0.0171;
    K_amortiguamiento_critico = 0.034268;
    K_sub_amortiguamiento = 0.0897;
    K_oscilatorio = 0.9356;
    
    % Transferencias a Lazo Abierto
    GH_total_lazo_abierto_Ksobre = series( K_sobre_amortiguamiento * G_total, H_total );
    GH_total_lazo_abierto_Kcritico = series( K_amortiguamiento_critico * G_total, H_total );
    GH_total_lazo_abierto_Ksub = series( K_sub_amortiguamiento * G_total, H_total );
    GH_total_lazo_abierto_Koscilatorio = series( K_oscilatorio * G_total, H_total );
    
    % Transferencias a Lazo Cerrado
    M_total_lazo_cerrado_Ksobre = feedback( K_sobre_amortiguamiento * G_total, H_total );
    M_total_lazo_cerrado_Kcritico = feedback( K_amortiguamiento_critico * G_total, H_total );
    M_total_lazo_cerrado_Ksub = feedback( K_sub_amortiguamiento * G_total, H_total );
    M_total_lazo_cerrado_Koscilatorio = feedback( K_oscilatorio * G_total, H_total );

    % Simulacion mediante simulink del modelo para obtener los resultados
    Matriz_resultados = sim( 'Punto1c_rozamiento_extra', 'SaveOutput', 'on' );
    Salida_Posicion_1c_sobre = Matriz_resultados.get( 'Salida_Posicion_1c_sobre' );
    Salida_Posicion_1c_critico = Matriz_resultados.get( 'Salida_Posicion_1c_critico' );
    Salida_Posicion_1c_sub = Matriz_resultados.get( 'Salida_Posicion_1c_sub' );
    Salida_Posicion_1c_oscilatorio = Matriz_resultados.get( 'Salida_Posicion_1c_oscilatorio' );
    Salida_Corriente_1c_sobre = Matriz_resultados.get( 'Salida_Corriente_1c_sobre' );
    Salida_Corriente_1c_critico = Matriz_resultados.get( 'Salida_Corriente_1c_critico' );
    Salida_Corriente_1c_sub = Matriz_resultados.get( 'Salida_Corriente_1c_sub' );
    Salida_Corriente_1c_oscilatorio = Matriz_resultados.get( 'Salida_Corriente_1c_oscilatorio' );
    Salida_Error_1c_sobre = Matriz_resultados.get( 'Salida_Error_1c_sobre' );
    Salida_Error_1c_critico = Matriz_resultados.get( 'Salida_Error_1c_critico' );
    Salida_Error_1c_sub = Matriz_resultados.get( 'Salida_Error_1c_sub' );
    Salida_Error_1c_oscilatorio = Matriz_resultados.get( 'Salida_Error_1c_oscilatorio' );
    Entrada_Posicion_1c = Matriz_resultados.get( 'Entrada_Posicion_1c' );
    
    cota_superior( 1:length(Entrada_Posicion_1c.time) ) = 1.02;
    cota_inferior( 1:length(Entrada_Posicion_1c.time) ) = 0.98;
    
    % Simulaciones con un escalon de posicion. Solo senales de salida y
    % entrada
    for numero_figura = 1 : 4
        
        switch numero_figura
            
            case 1  % Sobreamortiguado
                
                M_total_lazo_cerrado = M_total_lazo_cerrado_Ksobre;
                Texto_Tipo_sistema = 'sobreamortiguado';
                Texto_Tipo_Respuesta = 'sobreamortiguada';
                Valor_K = K_sobre_amortiguamiento;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Soreamortiguados_10Bm' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Sobreamortiguada_Escalon_10Bm' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Sobreamortiguada_Escalon_10Bm' ];
                Datos_Salida = Salida_Posicion_1c_sobre.signals.values;
                Datos_Error = Salida_Error_1c_sobre.signals.values;
                Datos_Corriente = Salida_Corriente_1c_sobre.signals.values;
                Datos_Entrada = Entrada_Posicion_1c.signals.values;
                Tiempo = Entrada_Posicion_1c.time;
                
            case 2  % Critico
                
                M_total_lazo_cerrado = M_total_lazo_cerrado_Kcritico;
                Texto_Tipo_sistema = 'amort. critico';
                Texto_Tipo_Respuesta = 'amort. critica';
                Valor_K = K_amortiguamiento_critico;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Criticos_10Bm' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Critica_Escalon_10Bm' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Critica_Escalon_10Bm' ];
                Datos_Salida = Salida_Posicion_1c_critico.signals.values;
                Datos_Error = Salida_Error_1c_critico.signals.values;
                Datos_Corriente = Salida_Corriente_1c_critico.signals.values;
                Datos_Entrada = Entrada_Posicion_1c.signals.values;
                Tiempo = Entrada_Posicion_1c.time;

            case 3  % Subamortiguado
                
                M_total_lazo_cerrado = M_total_lazo_cerrado_Ksub;
                Texto_Tipo_sistema = 'subamortiguado';
                Texto_Tipo_Respuesta = 'subamortiguada';
                Valor_K = K_sub_amortiguamiento;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Subamortiguados_10Bm' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Subamortiguada_Escalon_10Bm' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Subamortiguada_Escalon_10Bm' ];
                Datos_Salida = Salida_Posicion_1c_sub.signals.values;
                Datos_Error = Salida_Error_1c_sub.signals.values;
                Datos_Corriente = Salida_Corriente_1c_sub.signals.values;
                Datos_Entrada = Entrada_Posicion_1c.signals.values;
                Tiempo = Entrada_Posicion_1c.time;
                
            case 4  % Oscilatoria
                
                M_total_lazo_cerrado = M_total_lazo_cerrado_Koscilatorio;
                Texto_Tipo_sistema = 'oscilatorio';
                Texto_Tipo_Respuesta = 'oscilatoria';
                Valor_K = K_oscilatorio;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Oscilatorio_10Bm' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Oscilatoria_Escalon_10Bm' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Oscilatoria_Escalon_10Bm' ];
                Datos_Salida = Salida_Posicion_1c_oscilatorio.signals.values;
                Datos_Error = Salida_Error_1c_oscilatorio.signals.values;
                Datos_Corriente = Salida_Corriente_1c_oscilatorio.signals.values;
                Datos_Entrada = Entrada_Posicion_1c.signals.values;
                Tiempo = Entrada_Posicion_1c.time;
                
        end
        
        % Ploteo de la ubicacion de los polos
        figure();
        pzmap( M_total_lazo_cerrado );
        title( [ 'Ubicacion de los polos a Lazo Cerrado para el sistema ' Texto_Tipo_sistema ] );
        legend( [ 'K =' num2str( Valor_K ) ], 'location', 'best' );
        xlabel( 'Eje real' );
        ylabel( 'Eje imaginario' );
        print( [ Nombre_Archivo_Polos '.jpg' ], '-djpeg90' );
        % Ploteo de las respuestas
        figure();
        plot( Tiempo, Datos_Salida, 'b', 'LineWidth', 2 );
        hold on;
        plot( Tiempo, cota_inferior, '--r', 'LineWidth', 1 );
        hold on
        plot( Tiempo, cota_superior, '--r', 'LineWidth', 1 );
        hold on
        plot( Tiempo, Datos_Entrada, 'r', 'LineWidth', 2 );
        grid on;
        figure();
        plot( Tiempo, Datos_Salida, 'b', 'LineWidth', 2 );
        hold on;
        plot( Tiempo, Datos_Error, 'k', 'LineWidth', 2 );
        hold on;
        plot( Tiempo, Datos_Entrada, 'r', 'LineWidth', 2 );
        grid on;
        title( [ 'Respuesta ' Texto_Tipo_Respuesta ' frente a un escalon de excitacion' ] );
        legend( [ 'K =' num2str( Valor_K ) ], 'location', 'best' );
        xlabel( 'Tiempo [s]' );
        ylabel( 'Amplitud' );
        legend( 'Salida', 'Error', 'Referencia' );
        print( [ Nombre_Archivo_Respuesta '.jpg' ], '-djpeg90' );
        hold off;
        % Ploteo de las corrientes
        figure();
        plot( Tiempo, Datos_Corriente, 'b', 'LineWidth', 2 );
        grid on;
        title( [ 'Corriente ' Texto_Tipo_Respuesta ' frente a un escalon de excitacion' ] );
        legend( [ 'K =' num2str( Valor_K ) ], 'location', 'best' );
        xlabel( 'Tiempo [s]' );
        ylabel( 'Amplitud' );
        print( [ Nombre_Archivo_Corriente '.jpg' ], '-djpeg90' );
        
    end