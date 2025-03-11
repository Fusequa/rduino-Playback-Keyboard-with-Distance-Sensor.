
 //Mini-keyboard: nagrywanie (przyciski + czujnik),
 //odtwarzanie, ultradŸwiêki, UART, 3 g³oœniki


#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


   //1. DEFINICJE PINÓW

#define Nuta1    PD2
#define Nuta2    PD3
#define Nuta3    PD4
#define Nagrywanie   PC1
#define Odtwarzanie     PC2
#define Glosnik1 PB1   // Timer1 
#define Glosnik2 PD5   // Software PWM
#define Glosnik3 PB3   // Software PWM
#define LED1_PIN     PB4
#define LED2_PIN     PB5
#define LED3_PIN     PD7
#define HC_TRIG_PIN  PC0
#define HC_ECHO_PIN  PB0
#define Limit_eventow   50


  // 2. STRUKTURA EVENT

typedef struct {
    uint32_t delta_czas_us; 
    bool note1, note2, note3;
} Event;
// zmienne czasu/nagrywania
static Event   events[Limit_eventow];
static int     Liczba_eventow      = 0;
static bool    nagrywanie_ON     = false;
static bool    Odtwarzanie_ON       = false;
static bool    Nuta1_wczesniej       = false; 
static bool    Nuta2_wczesniej       = false;
static bool    Nuta3_wczesniej       = false;

// Zmienne czasu/odtwarzania
static uint32_t wczesniej_event_us    = 0;
static int      playback_Pamieta       = 0;
static uint32_t playback_czas_On_us= 0;
static uint32_t Czas_kolejnego_eventu_us    = 0;


  // 3. TIMER0 -> LICZENIE CZASU (micros, millis)

volatile static uint32_t przeregulowanie_T0 = 0;
ISR(TIMER0_OVF_vect) { przeregulowanie_T0++; }

uint32_t micros(void)
{
    uint32_t m;
    uint8_t t, s = SREG;
    cli();
    m = przeregulowanie_T0;
    t = TCNT0;
    SREG = s;
    return (m * 256UL * 4UL) + (t * 4UL);
}
uint32_t millis(void) { return micros() / 1000UL; }


 //  4. UART (do wyœwietlania odleglosci za pomoca czujnika ultradzwiekowego)

#define BAUD 9600
#define UBRR_VALUE ((F_CPU/(16UL*BAUD))-1)

void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t) UBRR_VALUE;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_transmit(char d) { while(!(UCSR0A & (1<<UDRE0))); UDR0 = d; }
void uart_print(const char *s) { while(*s) uart_transmit(*s++); }
void uart_println(const char *s){ uart_print(s); uart_transmit('\n'); }


  // 5. TIMER1 -> SPRZÊTOWE PWM (Speaker1)

void pwm_timer1_init(void)
{
    DDRB  |= (1 << Glosnik1);
    TCCR1A = (1 << WGM11) | (1 << COM1A1);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10);
    ICR1   = 2000; // ~125 Hz
}
void setSpeaker1Tone(bool on)
{
    if(on){
        OCR1A   = ICR1 / 8; 
        TCCR1A |= (1 << COM1A1);
    } else {
        TCCR1A &= ~(1 << COM1A1);
        PORTB  &= ~(1 << Glosnik1);
    }
}


  // 6. SOFTWARE PWM (Speaker2, Speaker3)

void updateSpeaker2_softwarePWM(bool active)
{
    static uint32_t start2=0; 
    uint32_t now=millis();
    if(active){
        if((now - start2) >= 6) start2=now;
        if((now - start2) < 1) PORTD|= (1<<Glosnik2);
        else                   PORTD&=~(1<<Glosnik2);
    } else {
        PORTD&=~(1<<Glosnik2);
        start2= now;
    }
}
void updateSpeaker3_softwarePWM(bool active)
{
    static uint32_t start3=0;
    uint32_t now=millis();
    if(active){
        if((now - start3) >= 8) start3=now;
        if((now - start3) < 1) PORTB|= (1<<Glosnik3);
        else                   PORTB&=~(1<<Glosnik3);
    } else {
        PORTB&=~(1<<Glosnik3);
        start3= now;
    }
}


  // 7. POMIAR HC-SR04

uint32_t Pomiar_pojedynczy_dystansu(void)
{
    PORTC&=~(1<<HC_TRIG_PIN); _delay_us(2);
    PORTC|= (1<<HC_TRIG_PIN); _delay_us(10);
    PORTC&=~(1<<HC_TRIG_PIN);

    uint32_t t0=micros();
    while(!(PINB&(1<<HC_ECHO_PIN))){ if(micros()-t0>30000UL)return 0; }
    uint32_t s= micros();
    while(PINB&(1<<HC_ECHO_PIN))  { if(micros()-s>30000UL)break; }
    return (micros()-s)/58;
}

uint32_t Stabilizowanie_dystansu(void)
{
    uint32_t sum=0; int valid=0;
    for(int i=0;i<5;i++){
        uint32_t d=Pomiar_pojedynczy_dystansu();
        if(d>0){ sum+=d; valid++; }
        _delay_ms(10);
    }
    return (valid==0)?999:(sum/valid);
}


 //  8. NAGRYWANIE I ODTWARZANIE

void addEvent(uint32_t delta, bool n1, bool n2, bool n3)
{
    if(Liczba_eventow<Limit_eventow){
        events[Liczba_eventow].delta_czas_us= delta;
        events[Liczba_eventow].note1=n1; 
        events[Liczba_eventow].note2=n2; 
        events[Liczba_eventow].note3=n3;
        Liczba_eventow++;
    }
}

void start_nagrywania(void)
{
    nagrywanie_ON=true; 
    Liczba_eventow=0; 
    wczesniej_event_us= micros();
    Nuta1_wczesniej=Nuta2_wczesniej=Nuta3_wczesniej=false;
}
void stop_nagrywania(void){ nagrywanie_ON=false; }

void sprawdz_stan(bool n1, bool n2, bool n3)
{
    if(n1!=Nuta1_wczesniej||n2!=Nuta2_wczesniej||n3!=Nuta3_wczesniej){
        uint32_t now_us= micros();
        addEvent(now_us-wczesniej_event_us,n1,n2,n3);
        wczesniej_event_us= now_us;
        Nuta1_wczesniej=n1; 
        Nuta2_wczesniej=n2; 
        Nuta3_wczesniej=n3;
    }
}

void startPlayback(void)
{
    if(!Liczba_eventow)return; 
    Odtwarzanie_ON=true; 
    playback_Pamieta=0;
    playback_czas_On_us= micros();
    Nuta1_wczesniej= events[0].note1;
    Nuta2_wczesniej= events[0].note2;
    Nuta3_wczesniej= events[0].note3;
    Czas_kolejnego_eventu_us= (Liczba_eventow>1)? events[1].delta_czas_us:0;
}
void stopPlayback(void)
{
    Odtwarzanie_ON=false;
    setSpeaker1Tone(false);
    updateSpeaker2_softwarePWM(false);
    updateSpeaker3_softwarePWM(false);
}
void updatePlayback(void)
{
    uint32_t now_us= micros();
    if(Odtwarzanie_ON && (now_us-playback_czas_On_us >= Czas_kolejnego_eventu_us)){
        playback_Pamieta++;
        if(playback_Pamieta>=Liczba_eventow){Odtwarzanie_ON=false;return;}
        Nuta1_wczesniej= events[playback_Pamieta].note1;
        Nuta2_wczesniej= events[playback_Pamieta].note2;
        Nuta3_wczesniej= events[playback_Pamieta].note3;
        Czas_kolejnego_eventu_us= events[playback_Pamieta].delta_czas_us+(now_us-playback_czas_On_us);
    }
}


  // 9. INICJALIZACJA 

void init_custom(void)
{
    // Przyciski nut
    DDRD &= ~((1<<Nuta1)|(1<<Nuta2)|(1<<Nuta3));
    PORTD|=  (1<<Nuta1)|(1<<Nuta2)|(1<<Nuta3);

    // Przyciski RECORD / PLAY
    DDRC &= ~((1<<Nagrywanie)|(1<<Odtwarzanie));
    PORTC|=  (1<<Nagrywanie)|(1<<Odtwarzanie);

    // LED
    DDRB|= (1<<LED1_PIN)|(1<<LED2_PIN);
    DDRD|= (1<<LED3_PIN);

    // HC-SR04
    DDRC|= (1<<HC_TRIG_PIN);
    DDRB&=~(1<<HC_ECHO_PIN);
    PORTB|=(1<<HC_ECHO_PIN);

    // G³oœniki
    DDRD|= (1<<Glosnik2);
    DDRB|= (1<<Glosnik3);

    TCCR0A=0; 
    TCCR0B=(1<<CS01)|(1<<CS00);
    TIMSK0=(1<<TOIE0);

    pwm_timer1_init();
    uart_init();
    sei();
}


  // 10. FUNKCJA G£ÓWNA

int main(void)
{
    init_custom();
    uint32_t przesz³y_odczyt_sensora=0; 
    static uint32_t distance=999;
    char buf[32];

    while(1)
    {
        // Stan manualnych przycisków
        bool n1=!(PIND&(1<<Nuta1));
        bool n2=!(PIND&(1<<Nuta2));
        bool n3=!(PIND&(1<<Nuta3));
        // RECORD, PLAY
        bool rec=!(PINC&(1<<Nagrywanie));
        bool ply=!(PINC&(1<<Odtwarzanie));

        // Odczyt czujnika co 100 ms
        uint32_t msNow= millis();
        if(msNow-przesz³y_odczyt_sensora>=100){
            przesz³y_odczyt_sensora= msNow;
            distance= Stabilizowanie_dystansu(); 
            sprintf(buf,"Dist=%lu",distance);
            uart_println(buf);
        }

        // Generowanie nut z czujnika
        bool sn1=false, sn2=false, sn3=false;
        bool l1=false, l2=false, l3=false;
        if(distance<=10){ sn1=sn2=sn3=true; l1=l2=l3=true;}
        else if(distance<=20){sn1=true; l1=true;}
        else if(distance<=30){sn2=true; l2=true;}
        else if(distance<=40){sn3=true; l3=true;}

        // LED
        if(l1) PORTB|= (1<<LED1_PIN); else PORTB&=~(1<<LED1_PIN);
        if(l2) PORTB|= (1<<LED2_PIN); else PORTB&=~(1<<LED2_PIN);
        if(l3) PORTD|= (1<<LED3_PIN); else PORTD&=~(1<<LED3_PIN);

        // £¹czenie przycisków manualnych i czujnikowych
        // Te stany bêdziemy te¿ nagrywaæ
        bool eff1 = (n1 || sn1);
        bool eff2 = (n2 || sn2);
        bool eff3 = (n3 || sn3);

        if(rec && !nagrywanie_ON) start_nagrywania();
        else if(!rec && nagrywanie_ON) stop_nagrywania();

        // Gdy nagrywamy sprawdzamy zmiany w (eff1, eff2, eff3)
        if(nagrywanie_ON) sprawdz_stan(eff1, eff2, eff3);

        if(ply && !Odtwarzanie_ON && Liczba_eventow>0) startPlayback();
        else if(!ply && Odtwarzanie_ON) stopPlayback();

        if(Odtwarzanie_ON) updatePlayback();

        // Jeœli odtwarzamy, stany nut bierzemy z eventów;
        // w przeciwnym razie z eff1..3
        bool final_n1 = Odtwarzanie_ON ? Nuta1_wczesniej : eff1;
        bool final_n2 = Odtwarzanie_ON ? Nuta2_wczesniej : eff2;
        bool final_n3 = Odtwarzanie_ON ? Nuta3_wczesniej : eff3;

        // G³oœniki
        setSpeaker1Tone(final_n1);
        updateSpeaker2_softwarePWM(final_n2);
        updateSpeaker3_softwarePWM(final_n3);

        _delay_ms(1);
    }
    return 0;
}
