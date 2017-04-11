%%%%%%%%%%%%% CALCULOS DEL TP DE MATLAB Y SIMULINK DE CONTROL

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% ADVERTENCIA: Este script genera todas las imagenes del punto 1a,      %
%%%  el control simple de posicion, que son aproximadamente 61. Por       %
%%%  lo tanto, se generaran todas esas figuras cuando se corra el codigo, %
%%%  ademas de simular el correspondiente archivo de simulink. Con lo     %
%%%  que puede demorar 1 o 2 minutos en completarse                       %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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
carpeta_salida = [ 'Imagenes/1a_' num2str( fecha_ejecucion(1) ) '_' num2str( fecha_ejecucion(2) ) '_' num2str( fecha_ejecucion(3) ) '_' num2str( fecha_ejecucion(4) ) '_' num2str( fecha_ejecucion(5) ) ];
mkdir( carpeta_salida );



%%%%%% EJERCICIO 1A: Control simple de posicion angular sin realimentacion de velocidad

    %%%% MODELADO DEL MOTOR
    Polo_electrico = [ La Ra ];
    Polo_mecanico = [ Jm Bm ];
    G_motor_denominador = conv( Polo_electrico, Polo_mecanico );
    G_motor_numerador = Kt;
    G_motor = tf( G_motor_numerador, G_motor_denominador );
    H_motor_numerador = Kb;
    H_motor_denominador = 1;
    H_motor = tf( H_motor_numerador, H_motor_denominador );
    M_motor = feedback( G_motor, H_motor );                  % Es la simplifacion de G y H del motor realimentado
     % Aqui se tiene el bloque simplificado del motor sin la caja reductora
    G_reductor = tf( 1, [ Caja_reductora 0 ] );
     % Al estar en serie, se utiliza la funcion "series" de Matlab, que permite
     % simplificar 2 transmitancias en cascada
     % A partir de ahora, se tiene en la cadena directa, el bloque K que genera
     % el lugar de raices, el amplicifador de ganancia para la tension de
     % armadura y el bloque que representa al motor con transferencia Tita/Va
     % Como Kpot1 = Kpot2, se plantea incorporarlos en la cadena directa G, de
     % modo que quede la realimentacion unitaria.
    M_motor_reductor = series( M_motor, G_reductor );
    G_total = M_motor_reductor * Kpot1 * Amplificador;
    H_total = tf( 1 );
     % Transferencia GH para realizar el analisis del lugar de raices y la
     % estabilidad
    GH_total_lazo_abierto = series( G_total, H_total );

    
    %%%% TRAZADO DEL LUGAR DE RAICES PARA HALLAR LOS VALORES CARACTERISTICOS DE K

    figure();
    rlocus( GH_total_lazo_abierto );
    title( 'Lugar de raices de GH a lazo abierto. Parametro variable: K' );
    print( [ carpeta_salida '/Lugar_Raices_Completo' '.jpg' ], '-djpeg90' );

     % Del grafico del lugar de raices, se obtienen los valores importantes
     % de K

    K_sobre_amortiguamiento = 0.0005;
    K_amortiguamiento_critico = 0.00108;
    K_sub_amortiguamiento = 0.003;
    K_oscilatorio = 0.15;

    % Transferencias a Lazo Abierto
    GH_total_lazo_abierto_Ksobre = series( K_sobre_amortiguamiento * G_total, H_total );
    GH_total_lazo_abierto_Kcritico = series( K_amortiguamiento_critico * G_total, H_total );
    GH_total_lazo_abierto_Ksub = series( K_sub_amortiguamiento * G_total, H_total );
    GH_total_lazo_abierto_Koscilatorio = series( K_oscilatorio * G_total, H_total );

    
     % Transferencia M = G / ( 1 + GH ) a lazo cerrado para corroborar el
     % analisis del lugar de raices en funcion del K elegido
    
    % Transferencias a Lazo Cerrado
    M_total_lazo_cerrado_Ksobre = feedback( K_sobre_amortiguamiento * G_total, H_total );
    M_total_lazo_cerrado_Kcritico = feedback( K_amortiguamiento_critico * G_total, H_total );
    M_total_lazo_cerrado_Ksub = feedback( K_sub_amortiguamiento * G_total, H_total );
    M_total_lazo_cerrado_Koscilatorio = feedback( K_oscilatorio * G_total, H_total );


    %%%% SIMULACIONES
    
    % Simulacion con Simulink para obtener los datos
    Matriz_resultados = sim( 'Punto1a_simple_posicion', 'SaveOutput', 'on' );
    Salida_Posicion_1a_sobre = Matriz_resultados.get( 'Salida_Posicion_1a_sobre' );
    Salida_Posicion_1a_critico = Matriz_resultados.get( 'Salida_Posicion_1a_critico' );
    Salida_Posicion_1a_sub = Matriz_resultados.get( 'Salida_Posicion_1a_sub' );
    Salida_Posicion_1a_oscilatorio = Matriz_resultados.get( 'Salida_Posicion_1a_oscilatorio' );
    Salida_Corriente_1a_sobre = Matriz_resultados.get( 'Salida_Corriente_1a_sobre' );
    Salida_Corriente_1a_critico = Matriz_resultados.get( 'Salida_Corriente_1a_critico' );
    Salida_Corriente_1a_sub = Matriz_resultados.get( 'Salida_Corriente_1a_sub' );
    Salida_Corriente_1a_oscilatorio = Matriz_resultados.get( 'Salida_Corriente_1a_oscilatorio' );
    Salida_Error_1a_sobre = Matriz_resultados.get( 'Salida_Error_1a_sobre' );
    Salida_Error_1a_critico = Matriz_resultados.get( 'Salida_Error_1a_critico' );
    Salida_Error_1a_sub = Matriz_resultados.get( 'Salida_Error_1a_sub' );
    Salida_Error_1a_oscilatorio = Matriz_resultados.get( 'Salida_Error_1a_oscilatorio' );
    Entrada_Posicion_1a = Matriz_resultados.get( 'Entrada_Posicion_1a' );
    
    cota_superior( 1:length(Entrada_Posicion_1a.time) ) = 1.02;
    cota_inferior( 1:length(Entrada_Posicion_1a.time) ) = 0.98;
    
    % Simulaciones con un escalon
    for numero_figura = 1 : 4

        switch numero_figura

            case 1  % Sobreamortiguado

                M_total_lazo_cerrado = M_total_lazo_cerrado_Ksobre;
                Texto_Tipo_sistema = 'sobreamortiguado';
                Texto_Tipo_Respuesta = 'sobreamortiguada';
                Valor_K = K_sobre_amortiguamiento;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Soreamortiguados' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Sobreamortiguada_Escalon' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Sobreamortiguada_Escalon' ];
                Datos_Salida = Salida_Posicion_1a_sobre.signals.values;
                Datos_Error = Salida_Error_1a_sobre.signals.values;
                Datos_Corriente = Salida_Corriente_1a_sobre.signals.values;
                Datos_Entrada = Entrada_Posicion_1a.signals.values;
                Tiempo = Salida_Posicion_1a_sobre.time;

            case 2  % Critico

                M_total_lazo_cerrado = M_total_lazo_cerrado_Kcritico;
                Texto_Tipo_sistema = 'amort. critico';
                Texto_Tipo_Respuesta = 'amort. critica';
                Valor_K = K_amortiguamiento_critico;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Criticos' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Critica_Escalon' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Critica_Escalon' ];
                Datos_Salida = Salida_Posicion_1a_critico.signals.values;
                Datos_Error = Salida_Error_1a_critico.signals.values;
                Datos_Corriente = Salida_Corriente_1a_critico.signals.values;
                Datos_Entrada = Entrada_Posicion_1a.signals.values;
                Tiempo = Salida_Posicion_1a_critico.time;

            case 3  % Subamortiguado

                M_total_lazo_cerrado = M_total_lazo_cerrado_Ksub;
                Texto_Tipo_sistema = 'subamortiguado';
                Texto_Tipo_Respuesta = 'subamortiguada';
                Valor_K = K_sub_amortiguamiento;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Subamortiguados' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Subamortiguada_Escalon' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Subamortiguada_Escalon' ];
                Datos_Salida = Salida_Posicion_1a_sub.signals.values;
                Datos_Error = Salida_Error_1a_sub.signals.values;
                Datos_Corriente = Salida_Corriente_1a_sub.signals.values;
                Datos_Entrada = Entrada_Posicion_1a.signals.values;
                Tiempo = Salida_Posicion_1a_sub.time;

            case 4  % Oscilatoria

                M_total_lazo_cerrado = M_total_lazo_cerrado_Koscilatorio;
                Texto_Tipo_sistema = 'oscilatorio';
                Texto_Tipo_Respuesta = 'oscilatoria';
                Valor_K = K_oscilatorio;
                Nombre_Archivo_Polos = [ carpeta_salida '/Polos_Oscilatorio' ];
                Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Oscilatoria_Escalon' ];
                Nombre_Archivo_Corriente = [ carpeta_salida '/Corriente_Oscilatoria_Escalon' ];
                Datos_Salida = Salida_Posicion_1a_oscilatorio.signals.values;
                Datos_Error = Salida_Error_1a_oscilatorio.signals.values;
                Datos_Corriente = Salida_Corriente_1a_oscilatorio.signals.values;
                Datos_Entrada = Entrada_Posicion_1a.signals.values;
                Tiempo = Salida_Posicion_1a_oscilatorio.time;

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

    pause;
    
    % Simulaciones con una senial triangular
    for indice = 1 : 5

        switch indice

            case 1

                frecuencia = 0.1;
                Archivo_Simulacion = 'Punto1a_triangular_0_1rad';

            case 2

                frecuencia = 0.476;
                Archivo_Simulacion = 'Punto1a_triangular_0_476rad';

            case 3

                frecuencia = 1.2;
                Archivo_Simulacion = 'Punto1a_triangular_1_2rad';

            case 4

                frecuencia = 3.54;
                Archivo_Simulacion = 'Punto1a_triangular_3_54rad';

            case 5

                frecuencia = 33.3;
                Archivo_Simulacion = 'Punto1a_triangular_33_3rad';

        end

        Matriz_resultados = sim( Archivo_Simulacion, 'SaveOutput', 'on' );
        Salida_Posicion_1a_sobre = Matriz_resultados.get( 'Salida_Posicion_1a_sobre' );
        Salida_Posicion_1a_critico = Matriz_resultados.get( 'Salida_Posicion_1a_critico' );
        Salida_Posicion_1a_sub = Matriz_resultados.get( 'Salida_Posicion_1a_sub' );
        Salida_Posicion_1a_oscilatorio = Matriz_resultados.get( 'Salida_Posicion_1a_oscilatorio' );
        Entrada_Posicion_1a = Matriz_resultados.get( 'Entrada_Posicion_1a' );

        for numero_figura = 1 : 4

            switch numero_figura

                case 1  % Sobreamortiguado

                    M_total_lazo_cerrado = M_total_lazo_cerrado_Ksobre;
                    Texto_Tipo_sistema = 'sobreamortiguado';
                    Texto_Tipo_Respuesta = 'sobreamortiguada';
                    Valor_K = K_sobre_amortiguamiento;
                    Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Sobreamortiguada_Triangular_' num2str( frecuencia ) ];
                    Datos_Salida = Salida_Posicion_1a_sobre.signals.values;
                    Datos_Entrada = Entrada_Posicion_1a.signals.values;
                    Tiempo = Salida_Posicion_1a_sobre.time;

                case 2  % Critico

                    M_total_lazo_cerrado = M_total_lazo_cerrado_Kcritico;
                    Texto_Tipo_sistema = 'amort. critico';
                    Texto_Tipo_Respuesta = 'amort. critica';
                    Valor_K = K_amortiguamiento_critico;
                    Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Critica_Triangular_' num2str( frecuencia ) ];
                    Datos_Salida = Salida_Posicion_1a_critico.signals.values;
                    Datos_Entrada = Entrada_Posicion_1a.signals.values;
                    Tiempo = Salida_Posicion_1a_critico.time;

                case 3  % Subamortiguado

                    M_total_lazo_cerrado = M_total_lazo_cerrado_Ksub;
                    Texto_Tipo_sistema = 'subamortiguado';
                    Texto_Tipo_Respuesta = 'subamortiguada';
                    Valor_K = K_sub_amortiguamiento;
                    Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Subamortiguada_Triangular_' num2str( frecuencia ) ];
                    Datos_Salida = Salida_Posicion_1a_sub.signals.values;
                    Datos_Entrada = Entrada_Posicion_1a.signals.values;
                    Tiempo = Salida_Posicion_1a_sub.time;

                case 4  % Oscilatoria

                    M_total_lazo_cerrado = M_total_lazo_cerrado_Koscilatorio;
                    Texto_Tipo_sistema = 'oscilatorio';
                    Texto_Tipo_Respuesta = 'oscilatoria';
                    Valor_K = K_oscilatorio;
                    Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Oscilatoria_Triangular_' num2str( frecuencia ) ];
                    Datos_Salida = Salida_Posicion_1a_oscilatorio.signals.values;
                    Datos_Entrada = Entrada_Posicion_1a.signals.values;
                    Tiempo = Salida_Posicion_1a_oscilatorio.time;

            end

            figure();
            plot( Tiempo, Datos_Salida, 'b', 'LineWidth', 2 );
            hold on;
            plot( Tiempo, Datos_Entrada, 'r', 'LineWidth', 2 );
            grid on;
            title( [ 'Respuesta ' Texto_Tipo_Respuesta ' frente a una triangular de excitacion' ] );
            legend( [ 'K = ' num2str( Valor_K ) char(10) 'Frecuencia = ' num2str( frecuencia ) ' rad/seg' ], 'location', 'best' );
            xlabel( 'Tiempo [s]' );
            ylabel( 'Amplitud' );
            print( [ Nombre_Archivo_Respuesta '.jpg' ], '-djpeg90' );
            hold off;

        end

    end


    % Simulaciones con una senial senoidal
    for indice = 1 : 5

        switch indice

            case 1

                frecuencia = 0.1;
                Archivo_Simulacion = 'Punto1a_senoidal_0_1rad';

            case 2

                frecuencia = 0.476;
                Archivo_Simulacion = 'Punto1a_senoidal_0_476rad';

            case 3

                frecuencia = 1.2;
                Archivo_Simulacion = 'Punto1a_senoidal_1_2rad';

            case 4

                frecuencia = 3.54;
                Archivo_Simulacion = 'Punto1a_senoidal_3_54rad';

            case 5

                frecuencia = 33.3;
                Archivo_Simulacion = 'Punto1a_senoidal_33_3rad';

        end

        Matriz_resultados = sim( Archivo_Simulacion, 'SaveOutput', 'on' );
        Salida_Posicion_1a_sobre = Matriz_resultados.get( 'Salida_Posicion_1a_sobre' );
        Salida_Posicion_1a_critico = Matriz_resultados.get( 'Salida_Posicion_1a_critico' );
        Salida_Posicion_1a_sub = Matriz_resultados.get( 'Salida_Posicion_1a_sub' );
        Salida_Posicion_1a_oscilatorio = Matriz_resultados.get( 'Salida_Posicion_1a_oscilatorio' );
        Entrada_Posicion_1a = Matriz_resultados.get( 'Entrada_Posicion_1a' );

        for numero_figura = 1 : 4

            switch numero_figura

                case 1  % Sobreamortiguado

                    M_total_lazo_cerrado = M_total_lazo_cerrado_Ksobre;
                    Texto_Tipo_sistema = 'sobreamortiguado';
                    Texto_Tipo_Respuesta = 'sobreamortiguada';
                    Valor_K = K_sobre_amortiguamiento;
                    Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Sobreamortiguada_senoidal_' num2str( frecuencia ) ];
                    Datos_Salida = Salida_Posicion_1a_sobre.signals.values;
                    Datos_Entrada = Entrada_Posicion_1a.signals.values;
                    Tiempo = Salida_Posicion_1a_sobre.time;

                case 2  % Critico

                    M_total_lazo_cerrado = M_total_lazo_cerrado_Kcritico;
                    Texto_Tipo_sistema = 'amort. critico';
                    Texto_Tipo_Respuesta = 'amort. critica';
                    Valor_K = K_amortiguamiento_critico;
                    Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Critica_senoidal_' num2str( frecuencia ) ];
                    Datos_Salida = Salida_Posicion_1a_critico.signals.values;
                    Datos_Entrada = Entrada_Posicion_1a.signals.values;
                    Tiempo = Salida_Posicion_1a_critico.time;

                case 3  % Subamortiguado

                    M_total_lazo_cerrado = M_total_lazo_cerrado_Ksub;
                    Texto_Tipo_sistema = 'subamortiguado';
                    Texto_Tipo_Respuesta = 'subamortiguada';
                    Valor_K = K_sub_amortiguamiento;
                    Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Subamortiguada_senoidal_' num2str( frecuencia ) ];
                    Datos_Salida = Salida_Posicion_1a_sub.signals.values;
                    Datos_Entrada = Entrada_Posicion_1a.signals.values;
                    Tiempo = Salida_Posicion_1a_sub.time;

                case 4  % Oscilatoria

                    M_total_lazo_cerrado = M_total_lazo_cerrado_Koscilatorio;
                    Texto_Tipo_sistema = 'oscilatorio';
                    Texto_Tipo_Respuesta = 'oscilatoria';
                    Valor_K = K_oscilatorio;
                    Nombre_Archivo_Respuesta = [ carpeta_salida '/Respuesta_Oscilatoria_senoidal_' num2str( frecuencia ) ];
                    Datos_Salida = Salida_Posicion_1a_oscilatorio.signals.values;
                    Datos_Entrada = Entrada_Posicion_1a.signals.values;
                    Tiempo = Salida_Posicion_1a_oscilatorio.time;

            end

            figure();
            plot( Tiempo, Datos_Salida, 'b', 'LineWidth', 2 );
            hold on;
            plot( Tiempo, Datos_Entrada, 'r', 'LineWidth', 2 );
            grid on;
            title( [ 'Respuesta ' Texto_Tipo_Respuesta ' frente a una senoidal de excitacion' ] );
            legend( [ 'K = ' num2str( Valor_K ) char(10) 'Frecuencia = ' num2str( frecuencia ) ' rad/seg' ], 'location', 'best' );
            xlabel( 'Tiempo [s]' );
            ylabel( 'Amplitud' );
            print( [ Nombre_Archivo_Respuesta '.jpg' ], '-djpeg90' );
            hold off;

        end

    end




    %%%% ANALISIS DEL ANCHO DE LA RESPUESTA FRECUENCIAL Y EL ANCHO DE BANDA DE LAS 4 RESPUESTAS OBTENIDAS EN FUNCION DE K

    % Diagramas de Bode a Lazo Abierto
    figure();
    bode( GH_total_lazo_abierto_Ksobre );
    hold on;
    bode( GH_total_lazo_abierto_Kcritico );
    hold on;
    bode( GH_total_lazo_abierto_Ksub );
    hold on;
    bode( GH_total_lazo_abierto_Koscilatorio );
    legend( [ 'K sobreamortiguado =' num2str( K_sobre_amortiguamiento ) ], [ 'K critico =' num2str( K_amortiguamiento_critico ) ], [ 'K subamortiguado =' num2str( K_sub_amortiguamiento ) ], [ 'K oscilatorio =' num2str( K_oscilatorio ) ], 'location', 'best' );
    grid on;
    title( 'Diagramas de Bode a Lazo Abierto' );
    print( [ carpeta_salida '/Bode_Lazo_Abierto.jpg' ],'-djpeg90');
    hold off;

    % Diagramas de Nyquist a Lazo Abierto
    figure();
    nyquist( GH_total_lazo_abierto_Ksobre );
    hold on;
    nyquist( GH_total_lazo_abierto_Kcritico );
    hold on;
    nyquist( GH_total_lazo_abierto_Ksub );
    hold on;
    nyquist( GH_total_lazo_abierto_Koscilatorio );
    legend( [ 'K sobreamortiguado =' num2str( K_sobre_amortiguamiento ) ], [ 'K critico =' num2str( K_amortiguamiento_critico ) ], [ 'K subamortiguado =' num2str( K_sub_amortiguamiento ) ], [ 'K oscilatorio =' num2str( K_oscilatorio ) ], 'location', 'best' );
    title( 'Diagramas de Nyquist a Lazo Abierto' );
    print( [ carpeta_salida '/Nyquist_Lazo_Abierto.jpg' ],'-djpeg90');
    hold off;

    % Diagramas de Nichols a Lazo Abierto
    figure();
    nichols( GH_total_lazo_abierto_Ksobre );
    hold on;
    nichols( GH_total_lazo_abierto_Kcritico );
    hold on;
    nichols( GH_total_lazo_abierto_Ksub );
    hold on;
    nichols( GH_total_lazo_abierto_Koscilatorio );
    legend( [ 'K sobreamortiguado =' num2str( K_sobre_amortiguamiento ) ], [ 'K critico =' num2str( K_amortiguamiento_critico ) ], [ 'K subamortiguado =' num2str( K_sub_amortiguamiento ) ], [ 'K oscilatorio =' num2str( K_oscilatorio ) ], 'location', 'best' );
    title( 'Diagramas de Nichols a Lazo Abierto' );
    print( [ carpeta_salida '/Nichols_Lazo_Abierto.jpg' ],'-djpeg90');
    hold off;

    % Margenes de ganancia y fase para cada caso
    figure();
    margin( GH_total_lazo_abierto_Ksobre );
    legend( [ 'K sobreamortiguado =' num2str( K_sobre_amortiguamiento ) ], 'location', 'best' );
    print( [ carpeta_salida '/Margen_Sobreamortiguado_Lazo_Abierto.jpg' ],'-djpeg90');
    figure();
    margin( GH_total_lazo_abierto_Kcritico );
    legend( [ 'K critico =' num2str( K_amortiguamiento_critico ) ], 'location', 'best' );
    print( [ carpeta_salida '/Margen_Critico_Lazo_Abierto.jpg' ],'-djpeg90');
    figure();
    margin( GH_total_lazo_abierto_Ksub );
    legend( [ 'K subamortiguado =' num2str( K_sub_amortiguamiento ) ], 'location', 'best' );
    print( [ carpeta_salida '/Margen_Subamortiguado_Lazo_Abierto.jpg' ],'-djpeg90');
    figure();
    margin( GH_total_lazo_abierto_Koscilatorio );
    legend( [ 'K oscilatorio =' num2str( K_oscilatorio ) ], 'location', 'best' );
    print( [ carpeta_salida '/Margen_Oscilatorio_Lazo_Abierto.jpg' ],'-djpeg90');

    % Diagramas de Bode a Lazo Cerrado, para determinar el ancho de banda
    figure();
    bode( M_total_lazo_cerrado_Ksobre );
    hold on;
    bode( M_total_lazo_cerrado_Kcritico );
    hold on;
    bode( M_total_lazo_cerrado_Ksub );
    hold on;
    bode( M_total_lazo_cerrado_Koscilatorio );
    legend( [ 'K sobreamortiguado =' num2str( K_sobre_amortiguamiento ) ], [ 'K critico =' num2str( K_amortiguamiento_critico ) ], [ 'K subamortiguado =' num2str( K_sub_amortiguamiento ) ], [ 'K oscilatorio =' num2str( K_oscilatorio ) ], 'location', 'best' );
    grid on;
    title( 'Diagramas de Bode a Lazo Cerrado' );
    print( [ carpeta_salida '/Bode_Lazo_Cerrado.jpg' ],'-djpeg90');
    hold off;


    %%%% COMPENSACION POR REALIMENTACION DE LAS VARIABLES DE ESTADO

    Matriz_resultados = sim( 'Realimentacion_VdE', 'SaveOutput', 'on' );
    Salida_Posicion_VdE = Matriz_resultados.get( 'Salida_Posicion_VdE' );
    Entrada_Posicion_VdE = Matriz_resultados.get( 'Entrada_Posicion_VdE' );

    % Grafico de la resuesta temporal para tomar el tiempo de setup
    [ salida, tiempo ] = step( M_total_lazo_cerrado_Ksub, 3 );
    cota_superior( 1:length(tiempo) ) = 1.02;
    cota_inferior( 1:length(tiempo) ) = 0.98;
    figure();
    plot( tiempo, salida, 'b', 'LineWidth', 1 );
    hold on
    plot( tiempo, cota_inferior, '--r', 'LineWidth', 1 );
    hold on
    plot( tiempo, cota_superior, '--r', 'LineWidth', 1 );
    title( 'Medicion del tiempo de setup para el sistema subamortiguado' );
    xlabel( 'Tiempo [s]' );
    ylabel( 'Amplitud' );
    legend( [ 'K subamortiguado = ' num2str( K_sub_amortiguamiento ) char(10) 'MOR = 10%' ], 'location', 'best' );
    grid on;
    print( [ carpeta_salida '/Tiempo_Setup.jpg' ],'-djpeg90');

    % El tiempo de setup da aproximadamente 1.9 Segundos tomando la banda del
    % 2%. Por lo tanto, el tiempo final es de 0.45 Segundos, ya que hay que
    % mejorarlo 4 veces. Ademas, el MOR debe ser de 4.3%


    %%% Realimentacion de las variables de estado para ubicar los polos en
    %   -9 +/- 9j. El otro polo puede quedar donde esta (en -114)
    A = [ -Ra/La -Kb/La 0; Kt/Jm -Bm/Jm 0; 0 1/Caja_reductora 0 ];
    B = [ 1/La; 0; 0 ];
    C = [ 0 0 1 ];
    D = 0;
    Q = [ B A*B A*A*B ];
    polos_a_lazo_cerrado = [ -9-9i -9+9i -114 ];
    matriz_k = acker( A, B, polos_a_lazo_cerrado );

    clear cota_superior;
    clear cota_inferior;
    cota_superior( 1:length(Salida_Posicion_VdE.time) ) = 1.02;
    cota_inferior( 1:length(Salida_Posicion_VdE.time) ) = 0.98;

    figure();
    plot( Salida_Posicion_VdE.time, Salida_Posicion_VdE.signals.values );
    hold on;
    plot( Salida_Posicion_VdE.time, cota_inferior, '--k', 'LineWidth', 1 );
    hold on
    plot( Salida_Posicion_VdE.time, cota_superior, '--k', 'LineWidth', 1 );
    hold on
    plot( Entrada_Posicion_VdE.time, Entrada_Posicion_VdE.signals.values, 'r' );
    title( 'Respuesta frente a un escalon de excitacion con realimentacion de Variables de Estado' );
    xlabel( 'Tiempo [s]' );
    ylabel( 'Amplitud' );
    legend( [ ' Ts = 0.45Seg ' char(10) 'MOR = 4.3%' ] );
    grid on;
    print( [ carpeta_salida '/Realimentado_VdE.jpg' ],'-djpeg90');
    hold off;
    
    
    
    %%%% EFECTO DE LA ZONA MUERTA

    Matriz_resultados = sim( 'Punto1a_zona_muerta', 'SaveOutput', 'on' );
    Salida_Posicion_1a_sin_zona_muerta = Matriz_resultados.get( 'Salida_Posicion_1a_sin_zona_muerta' );
    Salida_Posicion_1a_con_zona_muerta_error = Matriz_resultados.get( 'Salida_Posicion_1a_con_zona_muerta_error' );
    Salida_Posicion_1a_con_zona_muerta = Matriz_resultados.get( 'Salida_Posicion_1a_con_zona_muerta' );
    Salida_Posicion_1a_sin_zona_muerta_error = Matriz_resultados.get( 'Salida_Posicion_1a_sin_zona_muerta_error' );
    Entrada_Posicion_1a_zona_muerta = Matriz_resultados.get( 'Entrada_Posicion_1a_zona_muerta' );

    figure();
    plot( Salida_Posicion_1a_sin_zona_muerta.time, Salida_Posicion_1a_sin_zona_muerta.signals.values, 'LineWidth', 1 );
    hold on;
    plot( Salida_Posicion_1a_sin_zona_muerta_error.time, Salida_Posicion_1a_sin_zona_muerta_error.signals.values, 'r', 'LineWidth', 1 );
    hold on;
    plot( Entrada_Posicion_1a_zona_muerta.time, Entrada_Posicion_1a_zona_muerta.signals.values, 'm', 'LineWidth', 1 );
    title( 'Señales sin el efecto de la zona muerta del motor' );
    xlabel( 'Tiempo [s]' );
    ylabel( 'Amplitud' );
    legend( 'Señal de salida', 'Señal de error', 'Señal de entrada', 'location', 'best' );
    grid on;
    print( [ carpeta_salida '/Zona_muerta_sin_efecto.jpg' ],'-djpeg90');
    hold off;

    figure();
    plot( Salida_Posicion_1a_con_zona_muerta.time, Salida_Posicion_1a_con_zona_muerta.signals.values, 'LineWidth', 1 );
    hold on;
    plot( Salida_Posicion_1a_con_zona_muerta_error.time, Salida_Posicion_1a_con_zona_muerta_error.signals.values, 'r', 'LineWidth', 1 );
    hold on;
    plot( Entrada_Posicion_1a_zona_muerta.time, Entrada_Posicion_1a_zona_muerta.signals.values, 'm', 'LineWidth', 1 );
    title( 'Señales con el efecto de la zona muerta del motor' );
    xlabel( 'Tiempo [s]' );
    ylabel( 'Amplitud' );
    legend( 'Señal de salida', 'Señal de error', 'Señal de entrada', 'location', 'best' );
    grid on;
    print( [ carpeta_salida '/Zona_muerta_efecto.jpg' ],'-djpeg90');
    hold off;