// 🛠️ **Variable global para controlar el PWM de los motores**
int pwmMotores = 65;  //  PWM (ajustable)**

// Declaraciones de funciones
void configurarPines();
void configurarPWM();
void configurarServo();
void girarDerecha();
void girarIzquierda();
void avanzar();
void detenerCarro();
void moverServo();
void delay_ms(int ms);

// Variables globales
int sensorIzquierda, sensorDerecha, sensorFrontal, sensorCalor;

void configurarPines() {
    // Configuración de pines para sensores
    DDRD &= ~(1 << PD2); // PD2 como entrada (sensor izquierda)
    DDRD &= ~(1 << PD3); // PD3 como entrada (sensor derecha)
    DDRD &= ~(1 << PD4); // PD4 como entrada (sensor frontal)
    DDRB &= ~(1 << PB0); // PB0 como entrada (sensor de calor)

    // Configuración de pines para puente H L298 (salidas)
    DDRB |= (1 << PB2);  // enA (PWM Motor Izquierdo)
    DDRB |= (1 << PB3);  // enB (PWM Motor Derecho)
    DDRD |= (1 << PD5);  // IN1 (Motor Izquierdo)
    DDRD |= (1 << PD6);  // IN2 (Motor Izquierdo)
    DDRD |= (1 << PD7);  // IN3 (Motor Derecho)
    DDRB |= (1 << PB4);  // IN4 (Motor Derecho)

    // Configuración de pines para el servomotor y la bomba
    DDRB |= (1 << PB1);  // Configura PB1 como salida (OC1A - señal PWM para servo)
    DDRC |= (1 << PC0);  // Configura PC0 como salida para la bomba
}

// Configurar PWM para los motores con frecuencia 490 Hz
void configurarPWM() {
    TCCR1A &= ~(1 << COM1A1);  // Asegurar que no está en modo servo antes de activar motores

    // Configurar PB2 (Timer1 OCR1B) - Fast PWM a 490 Hz
    TCCR1A |= (1 << COM1B1) | (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1B = pwmMotores;  // **Usa la variable global**

    // Configurar PB3 (Timer2 OCR2A) - Fast PWM a 490 Hz
    TCCR2A = (1 << COM2A1) | (1 << WGM20) | (1 << WGM21);
    TCCR2B = (1 << CS22);
    OCR2A = pwmMotores;  // **Usa la variable global**
}

// Configurar PWM para el servomotor (50 Hz)
void configurarServo() {
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11);
    ICR1 = 39999; // TOP para 50 Hz (20 ms de periodo)
}

// Detener el carrito
void detenerCarro() {
    OCR1B = 0;
    OCR2A = 0;
    PORTD &= ~(1 << PD5);
    PORTD &= ~(1 << PD6);
    PORTD &= ~(1 << PD7);
    PORTB &= ~(1 << PB4);
}

// Girar a la derecha
void girarDerecha() {
    OCR1B = pwmMotores;  
    OCR2A = 0;   

    PORTD |= (1 << PD5);  
    PORTD &= ~(1 << PD6); 

    PORTD &= ~(1 << PD7); 
    PORTB &= ~(1 << PB4);
}

// Girar a la izquierda
void girarIzquierda() {
    OCR2A = pwmMotores;  
    OCR1B = 0;   

    PORTD |= (1 << PD7);  
    PORTB &= ~(1 << PB4); 

    PORTD &= ~(1 << PD5); 
    PORTD &= ~(1 << PD6);
}

// Avanzar lentamente hacia el fuego
void avanzar() {
    OCR1B = pwmMotores;  
    OCR2A = pwmMotores;  

    PORTD |= (1 << PD5);  
    PORTD &= ~(1 << PD6);

    PORTD |= (1 << PD7);  
    PORTB &= ~(1 << PB4);
}

// Barrido del servomotor y activación de la bomba
void moverServo() {
    configurarServo();
    PORTC |= (1 << PC0); // Activa la bomba

    for (int n = 0; n <= 1; n++) {
        OCR1A = (1000 - 500) * (39999.0 / 20000.0); // Posición mínima (0°)
        delay_ms(300);

        OCR1A = (3000 - 500) * (39999.0 / 20000.0); // Posición máxima (180°)
        delay_ms(300);
    }

    PORTC &= ~(1 << PC0); // Apaga la bomba

    configurarPWM();  // 🔄 **Regresar Timer1 al modo Motores**
}

// Delay sin bloquear el código
void delay_ms(int ms) {
    for (int i = 0; i < ms; i++) {
        for (int j = 0; j < 16000; j++) {
            asm volatile("nop");
        }
    }
}

void setup() {
    configurarPines();
    configurarPWM();
    Serial.begin(9600);
}

void loop() {
    // Leer sensores y corregir valores (0 = fuego, 4 = no fuego)
    sensorIzquierda = !(PIND & (1 << PD2));  
    sensorDerecha = !(PIND & (1 << PD3));  
    sensorFrontal = !(PIND & (1 << PD4));
    sensorCalor = !(PINB & (1 << PB0)); // Sensor de calor en PB0

    // Imprimir estado de los sensores en el monitor serie
    Serial.print("Izquierda: ");
    Serial.print(sensorIzquierda ? "🔥" : "✅");
    Serial.print(" | Derecha: ");
    Serial.print(sensorDerecha ? "🔥" : "✅");
    Serial.print(" | Frontal: ");
    Serial.print(sensorFrontal ? "🔥" : "✅");
    Serial.print(" | Calor: ");
    Serial.println(sensorCalor ? "🔥🔥 DETENIENDO" : "✅");
    


    if (!sensorIzquierda && !sensorDerecha && !sensorFrontal) {
        Serial.println("[❌] No hay fuego detectado. Deteniendo carrito.");
        
        detenerCarro();
        return;
    }

    if (sensorDerecha && !sensorFrontal) {
        Serial.println("[➡️] Fuego a la derecha. Girando...");
        
        girarDerecha();
        return;
    } 

    if (sensorIzquierda && !sensorFrontal) {
        Serial.println("[⬅️] Fuego a la izquierda. Girando...");
        
        girarIzquierda();
        return;
    }

    if (sensorFrontal) {
        Serial.println("[🚗] Fuego alineado. Avanzando hasta distancia segura...");
      

        while (!sensorCalor) {
            avanzar();
            delay_ms(100);
            sensorCalor = !(PINB & (1 << PB0));
        }

        detenerCarro();
        Serial.println("[🛑] Distancia segura alcanzada. Detenido.");
      
        Serial.println("[💦] Activando bomba y realizando barrido...");
        
        moverServo();
        Serial.println("[✅] Barrido completado. Listo para siguiente detección.");
        
        return;
    }
}
