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
carpeta_salida = [ 'Imagenes/1d_' num2str( fecha_ejecucion(1) ) '_' num2str( fecha_ejecucion(2) ) '_' num2str( fecha_ejecucion(3) ) '_' num2str( fecha_ejecucion(4) ) '_' num2str( fecha_ejecucion(5) ) ];
mkdir( carpeta_salida );



%%%%%% EJERCICIO 1D: Control de posicion simple con realimentacion de la
%%%%%% velocidad


    %%%% MODELADO DEL MOTOR
    Polo_electrico = [ La Ra ];
    Polo_mecanico = [ Jm Bm ];
    G_motor_denominador = conv( Polo_electrico, Polo_mecanico );
    G_motor_numerador = Kt;
    G_motor = tf( G_motor_numerador, G_motor_denominador );
    H_motor_numerador = Kb;
    H_motor_denominador = 1;
    H_motor = tf( H_motor_numerador, H_motor_denominador );
    M_motor = feedback( G_motor, H_motor );
    G_reductor = tf( 1, [ Caja_reductora 0 ] );
    M_motor_reductor = series( M_motor, G_reductor );
    G_total = Amplificador * M_motor_reductor;
    H_velocidad_numerador = conv( Ktac * Caja_reductora, integrador );
    H_velocidad_denominador = 1;
    H_velocidad = tf( H_velocidad_numerador, H_velocidad_denominador );
    H_posicion_numerador = Kpot2;
    H_posicion_denominador = 1;
    H_posicion = tf( H_posicion_numerador, H_posicion_denominador );
    H_total = parallel( H_posicion, H_velocidad );
    GH_total_lazo_abierto = series( G_total, H_total );

    % Trazado del lugar de raices
    figure();
    rlocus( GH_total_lazo_abierto );
    title( 'Lugar de raices de GH a lazo abierto con real. de velocidad. Parametro variable: K' );
    print( [ carpeta_salida '/Lugar_Raices_Completo_real_velocidad' '.jpg' ], '-djpeg90' );
    
    % Valores obtenidos del lugar de raices
    K_sobre_amortiguamiento = 0.0007;
    K_amortiguamiento_critico = 0.0014;
    K_sub_amortiguamiento = 0.0102;
    
    % Transferencias a Lazo Cerrado
    M_total_lazo_cerrado_Ksobre = feedback( K_sobre_amortiguamiento * G_total, H_total );
    M_total_lazo_cerrado_Kcritico = feedback( K_amortiguamiento_critico * G_total, H_total );
    M_total_lazo_cerrado_Ksub = feedback( K_sub_amortiguamiento * G_total, H_total );

    % Simulacion mediante simulink del modelo para obtener los resultados
    Matriz_resultados = sim( 'Punto1d_real_velocidad', 'SaveOutput', 'on' );
    Salida_Posicion_1d_sobre = Matriz_resultados.get( 'Salida_Posicion_1d_sobre' );
    Salida_Posicion_1d_critico = Matriz_resultados.get( 'Salida_Posicion_1d_critico' );
    Salida_Posicion_1d_sub = Matriz_resultados.get( 'Salida_Posicion_1d_sub' );
    Salida_Corriente_1d_sobre = Matriz_resultados.get( 'Salida_Corriente_1d_sobre' );
    Salida_Corriente_1d_critico = Matriz_resultados.get( 'Salida_Corriente_1d_critico' );
    Salida_Corriente_1d_sub = Matriz_resultados.get( 'Salida_Corriente_1d_sub' );
    Salida_Error_1d_sobre = Matriz_resultados.get( 'Salida_Error_1d_sobre' );
    Salida_Error_1d_critico = Matriz_resultados.get( 'Salida_Error_1d_critico' );
    Salida_Error_1d_sub = Matriz_resultados.get( 'Salida_Error_1d_sub' );
    Entrada_Posicion_1d = Matriz_resultados.get( 'Entrada_Posicion_1d' );
    
    cota_superior( 1:length(Entrada_Posicion_1d.time) ) = 1.02;
    cota_inferior( 1:length(Entrada_Posicion_1d.time) ) = 0.98;
    
    % Simulaciones con un escalon de posicion. Solo senales de salida y
    % entrada
    for numero_figura = 1 : 3
        
        switch numero_figura
            
            case 1  % Sobreamortiguado
                
                M_total_lazo_cerrado = M_total_lazo_cerrado_Ksobre;
                Texto_Tipo_sistema = 'sobreamortiguado';
                Texto_Tipo_Respuesta = 'sobreamortiguada';
                Valor_K = K_sobre_amortiguamiento;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Sobreamortiguada_Escalon_real_velocidad' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Sobreamortiguada_Escalon_real_velocidad' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Sobreamortiguada_Escalon_real_velocidad' ];
                Datos_Salida = Salida_Posicion_1d_sobre.signals.values;
                Datos_Error = Salida_Error_1d_sobre.signals.values;
                Datos_Corriente = Salida_Corriente_1d_sobre.signals.values;
                Datos_Entrada = Entrada_Posicion_1d.signals.values;
                Tiempo = Entrada_Posicion_1d.time;
                
            case 2  % Critico
                
                M_total_lazo_cerrado = M_total_lazo_cerrado_Kcritico;
                Texto_Tipo_sistema = 'amort. critico';
                Texto_Tipo_Respuesta = 'amort. critica';
                Valor_K = K_amortiguamiento_critico;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Critica_Escalon_real_velocidad' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Critica_Escalon_real_velocidad' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Critica_Escalon_real_velocidad' ];
                Datos_Salida = Salida_Posicion_1d_critico.signals.values;
                Datos_Error = Salida_Error_1d_critico.signals.values;
                Datos_Corriente = Salida_Corriente_1d_critico.signals.values;
                Datos_Entrada = Entrada_Posicion_1d.signals.values;
                Tiempo = Entrada_Posicion_1d.time;

            case 3  % Subamortiguado
                
                M_total_lazo_cerrado = M_total_lazo_cerrado_Ksub;
                Texto_Tipo_sistema = 'subamortiguado';
                Texto_Tipo_Respuesta = 'subamortiguada';
                Valor_K = K_sub_amortiguamiento;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Subamortiguada_Escalon_real_velocidad' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Subamortiguada_Escalon_real_velocidad' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Subamortiguada_Escalon_real_velocidad' ];
                Datos_Salida = Salida_Posicion_1d_sub.signals.values;
                Datos_Error = Salida_Error_1d_sub.signals.values;
                Datos_Corriente = Salida_Corriente_1d_sub.signals.values;
                Datos_Entrada = Entrada_Posicion_1d.signals.values;
                Tiempo = Entrada_Posicion_1d.time;
                
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
    
    
    
    
    
%%%%%% EJERCICIO 1D: Idem anterior, disminuyendo Ktac = 0.1 * Ktac


    %%%% MODELADO DEL MOTOR
    Polo_electrico = [ La Ra ];
    Polo_mecanico = [ Jm Bm ];
    G_motor_denominador = conv( Polo_electrico, Polo_mecanico );
    G_motor_numerador = Kt;
    G_motor = tf( G_motor_numerador, G_motor_denominador );
    H_motor_numerador = Kb;
    H_motor_denominador = 1;
    H_motor = tf( H_motor_numerador, H_motor_denominador );
    M_motor = feedback( G_motor, H_motor );
    G_reductor = tf( 1, [ Caja_reductora 0 ] );
    M_motor_reductor = series( M_motor, G_reductor );
    G_total = Amplificador * M_motor_reductor;
    H_velocidad_numerador = conv( 0.1 * Ktac * Caja_reductora, integrador );
    H_velocidad_denominador = 1;
    H_velocidad = tf( H_velocidad_numerador, H_velocidad_denominador );
    H_posicion_numerador = Kpot2;
    H_posicion_denominador = 1;
    H_posicion = tf( H_posicion_numerador, H_posicion_denominador );
    H_total_01Ktac = parallel( H_posicion, H_velocidad );
    GH_total_lazo_abierto_01Ktac = series( G_total, H_total_01Ktac );

    % Trazado del lugar de raices
    figure();
    rlocus( GH_total_lazo_abierto_01Ktac );
    title( 'Lugar de raices de GH a lazo abierto con real. de velocidad y 0.1*Ktac. Parametro variable: K' );
    print( [ carpeta_salida '/Lugar_Raices_Completo_real_velocidad_01Ktac' '.jpg' ], '-djpeg90' );
    
    % Valores obtenidos del lugar de raices
    K_sobre_amortiguamiento_01Ktac = 0.0005;
    K_amortiguamiento_critico_01Ktac = 0.001108;
    K_sub_amortiguamiento_01Ktac = 0.0032;

    % Transferencias a Lazo Cerrado
    M_total_lazo_cerrado_Ksobre_01Ktac = feedback( K_sobre_amortiguamiento_01Ktac * G_total, H_total_01Ktac );
    M_total_lazo_cerrado_Kcritico_01Ktac = feedback( K_amortiguamiento_critico_01Ktac * G_total, H_total_01Ktac );
    M_total_lazo_cerrado_Ksub_01Ktac = feedback( K_sub_amortiguamiento_01Ktac * G_total, H_total_01Ktac );

    % Simulacion mediante simulink del modelo para obtener los resultados
    Matriz_resultados_01Ktac = sim( 'Punto1d_real_velocidad_01Ktac', 'SaveOutput', 'on' );
    Salida_Posicion_1d_sobre_01Ktac = Matriz_resultados_01Ktac.get( 'Salida_Posicion_1d_sobre' );
    Salida_Posicion_1d_critico_01Ktac = Matriz_resultados_01Ktac.get( 'Salida_Posicion_1d_critico' );
    Salida_Posicion_1d_sub_01Ktac = Matriz_resultados_01Ktac.get( 'Salida_Posicion_1d_sub' );
    Salida_Corriente_1d_sobre_01Ktac = Matriz_resultados_01Ktac.get( 'Salida_Corriente_1d_sobre' );
    Salida_Corriente_1d_critico_01Ktac = Matriz_resultados_01Ktac.get( 'Salida_Corriente_1d_critico' );
    Salida_Corriente_1d_sub_01Ktac = Matriz_resultados_01Ktac.get( 'Salida_Corriente_1d_sub' );
    Salida_Error_1d_sobre_01Ktac = Matriz_resultados_01Ktac.get( 'Salida_Error_1d_sobre' );
    Salida_Error_1d_critico_01Ktac = Matriz_resultados_01Ktac.get( 'Salida_Error_1d_critico' );
    Salida_Error_1d_sub_01Ktac = Matriz_resultados_01Ktac.get( 'Salida_Error_1d_sub' );
    Entrada_Posicion_1d_01Ktac = Matriz_resultados_01Ktac.get( 'Entrada_Posicion_1d' );
    
    clear cota_superior;
    clear cota_inferior;
    
    cota_superior( 1:length(Entrada_Posicion_1d_01Ktac.time) ) = 1.02;
    cota_inferior( 1:length(Entrada_Posicion_1d_01Ktac.time) ) = 0.98;
    
    % Simulaciones con un escalon de posicion. Solo senales de salida y
    % entrada
    for numero_figura = 1 : 3
        
        switch numero_figura
            
            case 1  % Sobreamortiguado
                
                M_total_lazo_cerrado_01Ktac = M_total_lazo_cerrado_Ksobre_01Ktac;
                Texto_Tipo_sistema = 'sobreamortiguado';
                Texto_Tipo_Respuesta = 'sobreamortiguada';
                Valor_K_01Ktac = K_sobre_amortiguamiento_01Ktac;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Soreamortiguados_real_velocidad_01Ktac' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Sobreamortiguada_Escalon_real_velocidad_01Ktac' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Sobreamortiguada_Escalon_real_velocidad_01Ktac' ];
                Datos_Salida_01Ktac = Salida_Posicion_1d_sobre_01Ktac.signals.values;
                Datos_Error_01Ktac = Salida_Error_1d_sobre_01Ktac.signals.values;
                Datos_Corriente_01Ktac = Salida_Corriente_1d_sobre_01Ktac.signals.values;
                Datos_Entrada_01Ktac = Entrada_Posicion_1d_01Ktac.signals.values;
                Tiempo_01Ktac = Entrada_Posicion_1d_01Ktac.time;
                
            case 2  % Critico
                
                M_total_lazo_cerrado_01Ktac = M_total_lazo_cerrado_Kcritico_01Ktac;
                Texto_Tipo_sistema = 'amort. critico';
                Texto_Tipo_Respuesta = 'amort. critica';
                Valor_K_01Ktac = K_amortiguamiento_critico_01Ktac;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Critica_Escalon_real_velocidad_01Ktac' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Critica_Escalon_real_velocidad_01Ktac' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Critica_Escalon_real_velocidad_01Ktac' ];
                Datos_Salida_01Ktac = Salida_Posicion_1d_critico_01Ktac.signals.values;
                Datos_Error_01Ktac = Salida_Error_1d_critico_01Ktac.signals.values;
                Datos_Corriente_01Ktac = Salida_Corriente_1d_critico_01Ktac.signals.values;
                Datos_Entrada_01Ktac = Entrada_Posicion_1d_01Ktac.signals.values;
                Tiempo_01Ktac = Entrada_Posicion_1d_01Ktac.time;

            case 3  % Subamortiguado
                
                M_total_lazo_cerrado_01Ktac = M_total_lazo_cerrado_Ksub_01Ktac;
                Texto_Tipo_sistema = 'subamortiguado';
                Texto_Tipo_Respuesta = 'subamortiguada';
                Valor_K_01Ktac = K_sub_amortiguamiento_01Ktac;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Subamortiguada_Escalon_real_velocidad_01Ktac' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Subamortiguada_Escalon_real_velocidad_01Ktac' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Subamortiguada_Escalon_real_velocidad_01Ktac' ];
                Datos_Salida_01Ktac = Salida_Posicion_1d_sub_01Ktac.signals.values;
                Datos_Error_01Ktac = Salida_Error_1d_sub_01Ktac.signals.values;
                Datos_Corriente_01Ktac = Salida_Corriente_1d_sub_01Ktac.signals.values;
                Datos_Entrada_01Ktac = Entrada_Posicion_1d_01Ktac.signals.values;
                Tiempo_01Ktac = Entrada_Posicion_1d_01Ktac.time;
                
        end
        
        % Ploteo de la ubicacion de los polos
        figure();
        pzmap( M_total_lazo_cerrado_01Ktac );
        title( [ 'Ubicacion de los polos a Lazo Cerrado para el sistema ' Texto_Tipo_sistema ] );
        legend( [ 'K =' num2str( Valor_K_01Ktac ) ], 'location', 'best' );
        xlabel( 'Eje real' );
        ylabel( 'Eje imaginario' );
        print( [ Nombre_Archivo_Polos '.jpg' ], '-djpeg90' );
        % Ploteo de las respuestas
        figure();
        plot( Tiempo_01Ktac, Datos_Salida_01Ktac, 'b', 'LineWidth', 2 );
        hold on;
        plot( Tiempo_01Ktac, cota_inferior, '--r', 'LineWidth', 1 );
        hold on
        plot( Tiempo_01Ktac, cota_superior, '--r', 'LineWidth', 1 );
        hold on
        plot( Tiempo_01Ktac, Datos_Entrada_01Ktac, 'r', 'LineWidth', 2 );
        grid on;
        figure();
        plot( Tiempo_01Ktac, Datos_Salida_01Ktac, 'b', 'LineWidth', 2 );
        hold on;
        plot( Tiempo_01Ktac, Datos_Error_01Ktac, 'k', 'LineWidth', 2 );
        hold on;
        plot( Tiempo_01Ktac, Datos_Entrada_01Ktac, 'r', 'LineWidth', 2 );
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
        plot( Tiempo_01Ktac, Datos_Corriente_01Ktac, 'b', 'LineWidth', 2 );
        grid on;
        title( [ 'Corriente ' Texto_Tipo_Respuesta ' frente a un escalon de excitacion' ] );
        legend( [ 'K =' num2str( Valor_K_01Ktac ) ], 'location', 'best' );
        xlabel( 'Tiempo [s]' );
        ylabel( 'Amplitud' );
        print( [ Nombre_Archivo_Corriente '.jpg' ], '-djpeg90' );
        
    end