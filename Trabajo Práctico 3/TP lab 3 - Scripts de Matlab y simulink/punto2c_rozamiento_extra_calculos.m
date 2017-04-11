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
Amplificador = 115;


% Demas cosas generales
fecha_ejecucion = clock;     % Se usa la fecha de ejecucion del script como nombre para la carpeta donde van las imagenes de salida
carpeta_salida = [ 'Imagenes/2c_' num2str( fecha_ejecucion(1) ) '_' num2str( fecha_ejecucion(2) ) '_' num2str( fecha_ejecucion(3) ) '_' num2str( fecha_ejecucion(4) ) '_' num2str( fecha_ejecucion(5) ) ];
mkdir( carpeta_salida );



%%%%%% EJERCICIO 2C: Control de velocidad con rozamiento extra

    %%%% MODELADO DEL MOTOR
    Polo_electrico = [ La Ra ];
    Polo_mecanico = [ Jm 10*Bm ];
    G_motor = tf( Kt, conv( Polo_electrico, Polo_mecanico ) );
    H_motor = tf( Kb, 1 );
    M_motor = feedback( G_motor, H_motor );
    G_total = M_motor * Amplificador;
    H_velocidad = tf( Ktac , 1 );
    GH_total_lazo_abierto = series( G_total, H_velocidad );
    M_total = feedback( G_total, H_velocidad );

        
    %%%% TRAZADO DEL LUGAR DE RAICES PARA HALLAR LOS VALORES CARACTERISTICOS DE K

    figure();
    rlocus( GH_total_lazo_abierto );
    title( 'Lugar de raices de GH a lazo abierto. Parametro variable: K' );
    print( [ carpeta_salida '/Lugar_Raices_Completo_velocidad_rozamiento_extra' '.jpg' ], '-djpeg90' );

    K_sobre_amortiguamiento = 0.0249;
    K_amortiguamiento_critico = 0.050337;
    K_sub_amortiguamiento = 0.257;

    % Transferencias a Lazo Abierto
    GH_total_lazo_abierto_Ksobre = series( K_sobre_amortiguamiento * G_total, H_velocidad );
    GH_total_lazo_abierto_Kcritico = series( K_amortiguamiento_critico * G_total, H_velocidad );
    GH_total_lazo_abierto_Ksub = series( K_sub_amortiguamiento * G_total, H_velocidad );

    % Transferencias a Lazo Cerrado
    M_total_lazo_cerrado_Ksobre = feedback( K_sobre_amortiguamiento * G_total, H_velocidad );
    M_total_lazo_cerrado_Kcritico = feedback( K_amortiguamiento_critico * G_total, H_velocidad );
    M_total_lazo_cerrado_Ksub = feedback( K_sub_amortiguamiento * G_total, H_velocidad );


    % Simulacion con Simulink para obtener los datos
    Matriz_resultados = sim( 'Punto2c_rozamiento_extra', 'SaveOutput', 'on' );
    Salida_Velocidad_2c_sobre = Matriz_resultados.get( 'Salida_Velocidad_2c_sobre' );
    Salida_Velocidad_2c_critico = Matriz_resultados.get( 'Salida_Velocidad_2c_critico' );
    Salida_Velocidad_2c_sub = Matriz_resultados.get( 'Salida_Velocidad_2c_sub' );
    Salida_Corriente_2c_sobre = Matriz_resultados.get( 'Salida_Corriente_2c_sobre' );
    Salida_Corriente_2c_critico = Matriz_resultados.get( 'Salida_Corriente_2c_critico' );
    Salida_Corriente_2c_sub = Matriz_resultados.get( 'Salida_Corriente_2c_sub' );
    Salida_Corriente_2c_oscilatorio = Matriz_resultados.get( 'Salida_Corriente_2c_oscilatorio' );
    Salida_Error_2c_sobre = Matriz_resultados.get( 'Salida_Error_2c_sobre' );
    Salida_Error_2c_critico = Matriz_resultados.get( 'Salida_Error_2c_critico' );
    Salida_Error_2c_sub = Matriz_resultados.get( 'Salida_Error_2c_sub' );
    Salida_Error_2c_oscilatorio = Matriz_resultados.get( 'Salida_Error_2c_oscilatorio' );
    Entrada_Velocidad_2c = Matriz_resultados.get( 'Entrada_Velocidad_2c' );



    % Simulaciones con un escalon
    for numero_figura = 1 : 3

        switch numero_figura

            case 1  % Sobreamortiguado

                M_total_lazo_cerrado = M_total_lazo_cerrado_Ksobre;
                Texto_Tipo_sistema = 'sobreamortiguado';
                Texto_Tipo_Respuesta = 'sobreamortiguada';
                Valor_K = K_sobre_amortiguamiento;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Soreamortiguados_rozamiento_extra' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Sobreamortiguada_Escalon_rozamiento_extra' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Sobreamortiguada_Escalon_rozamiento_extra' ];
                Datos_Salida = Salida_Velocidad_2c_sobre.signals.values;
                Datos_Error = Salida_Error_2c_sobre.signals.values;
                Datos_Corriente = Salida_Corriente_2c_sobre.signals.values;
                Datos_Entrada = Entrada_Velocidad_2c.signals.values;
                Tiempo = Salida_Velocidad_2c_sobre.time;

            case 2  % Critico

                M_total_lazo_cerrado = M_total_lazo_cerrado_Kcritico;
                Texto_Tipo_sistema = 'amort. critico';
                Texto_Tipo_Respuesta = 'amort. critica';
                Valor_K = K_amortiguamiento_critico;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Criticos_rozamiento_extra' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Critica_Escalon_rozamiento_extra' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Critica_Escalon_rozamiento_extra' ];
                Datos_Salida = Salida_Velocidad_2c_critico.signals.values;
                Datos_Error = Salida_Error_2c_critico.signals.values;
                Datos_Corriente = Salida_Corriente_2c_critico.signals.values;
                Datos_Entrada = Entrada_Velocidad_2c.signals.values;
                Tiempo = Salida_Velocidad_2c_critico.time;

            case 3  % Subamortiguado

                M_total_lazo_cerrado = M_total_lazo_cerrado_Ksub;
                Texto_Tipo_sistema = 'subamortiguado';
                Texto_Tipo_Respuesta = 'subamortiguada';
                Valor_K = K_sub_amortiguamiento;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Subamortiguados_rozamiento_extra' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Subamortiguada_Escalon_rozamiento_extra' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Subamortiguada_Escalon_rozamiento_extra' ];
                Datos_Salida = Salida_Velocidad_2c_sub.signals.values;
                Datos_Error = Salida_Error_2c_sub.signals.values;
                Datos_Corriente = Salida_Corriente_2c_sub.signals.values;
                Datos_Entrada = Entrada_Velocidad_2c.signals.values;
                Tiempo = Salida_Velocidad_2c_sub.time;

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
        hold off;

    end