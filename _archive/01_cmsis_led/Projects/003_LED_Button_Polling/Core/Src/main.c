/*
 * main.c (Projekt 6: Non-Blocking State Machine)
 * Ziel: Ein Lauflicht blinken lassen, WÄHREND gleichzeitig
 * ein Taster eingelesen wird.
 * Framework: CMSIS
 * MCU: STM32F103C6
 */

#include "stm32f103x6.h"
#include <stdint.h>

// Global Variables
extern volatile uint32_t g_msTicks; // Aus stm32f1xx_it.c

/* --- Prototypen --- */
void GPIO_Init_All(void);
void LED_Modus_Christmas_Update(void); // Unsere neue "non-blocking" Funktion
void Button_Check(void);             // Unsere neue Button-Funktion

/* --- Globale "Konstanten" für unser Lauflicht --- */
// (Wir definieren die Pins hier, um sie leichter zu ändern)
#define LED_PIN_0_PORT   GPIOC
#define LED_PIN_0_BIT_S  GPIO_BSRR_BR13 // Active-Low AN
#define LED_PIN_0_BIT_R  GPIO_BSRR_BS13 // Active-Low AUS

#define LED_PIN_1_PORT   GPIOA
#define LED_PIN_1_BIT_S  GPIO_BSRR_BS0
#define LED_PIN_1_BIT_R  GPIO_BSRR_BR0

#define LED_PIN_2_PORT   GPIOA
#define LED_PIN_2_BIT_S  GPIO_BSRR_BS10
#define LED_PIN_2_BIT_R  GPIO_BSRR_BR10

#define LED_PIN_3_PORT   GPIOB
#define LED_PIN_3_BIT_S  GPIO_BSRR_BS9
#define LED_PIN_3_BIT_R  GPIO_BSRR_BR9

/* --- Hauptprogramm --- */
int main(void) {

    // 1. System-Init (für g_msTicks)
    SysTick_Config(SystemCoreClock / 1000); // 1ms Tick

    // 2. Peripherie initialisieren
    GPIO_Init_All();

    // 3. Hauptschleife (Der "multitaskingfähige Koch")
    while (1) {

        /*
         * Diese Schleife läuft jetzt Millionen Mal pro Sekunde!
         * Wir "blockieren" nie wieder.
         */

        // Aufgabe 1: Prüfe den Taster (Passiert 10000x pro Sekunde)
        Button_Check();

        // Aufgabe 2: Aktualisiere das Lauflicht (Passiert 10000x pro Sekunde)
        LED_Modus_Christmas_Update();
    }
}


/* --- Implementierungen --- */

/**
 * @brief Prüft den Taster (PB0) und schaltet die Board-LED (PC13).
 * Diese Funktion ist "non-blocking". Sie kehrt SOFORT zurück.
 */
void Button_Check(void) {
    if ((GPIOB->IDR & GPIO_IDR_IDR0) == 0) {
        // Button gedrückt -> LED AN (Active-Low)
        LED_PIN_0_PORT->BSRR = LED_PIN_0_BIT_S;
    } else {
        // Button losgelassen -> LED AUS (Active-Low)
        LED_PIN_0_PORT->BSRR = LED_PIN_0_BIT_R;
    }
}

/**
 * @brief Führt EINEN Schritt des "Christmas"-Lauflichts aus.
 * Dies ist unsere "State Machine".
 */
void LED_Modus_Christmas_Update(void) {

    // 1. Die "Erinnerungs-Variablen" (Dank 'static' überleben sie)

    // Speichert den "Wecker" (Wann war der letzte Schritt?)
    static uint32_t last_tick = 0;

    // Speichert den "Fortschritt" (Welcher Schritt ist der nächste?)
    static int state = 0; // 0 = Schritt 1, 1 = Schritt 2, etc.

    // 2. Der "Timer" (Unser Wecker)
    const uint32_t delay = 300; // 300ms pro Schritt

    // Prüfe, ob der Wecker geklingelt hat:
    // "Ist die Zeit (g_msTicks) MEHR als 300ms nach 'last_tick'?"
    if ((g_msTicks - last_tick) < delay) {
        // NEIN: Wecker läuft noch. Nichts tun.
        // SOFORT ZURÜCKKEHREN (non-blocking!)
        return;
    }

    // JA: Der Wecker hat geklingelt! Zeit für den nächsten Schritt.

    // 3. Wecker "zurücksetzen": Merke dir die *jetzige* Zeit für den *nächsten* Wartezyklus.
    last_tick = g_msTicks;

    // 4. Der "Fortschritts-Manager" (Die Logik)
    // Führe den nächsten Schritt basierend auf 'state' aus.
    switch (state) {
        case 0: // Schritt 0: LED 1 AN
            LED_PIN_1_PORT->BSRR = LED_PIN_1_BIT_S;
            break;

        case 1: // Schritt 1: LED 2 AN
            LED_PIN_2_PORT->BSRR = LED_PIN_2_BIT_S;
            break;

        case 2: // Schritt 2: LED 3 AN
            LED_PIN_3_PORT->BSRR = LED_PIN_3_BIT_S;
            break;

        case 3: // Schritt 3: Pause (alle AN)
            // (Nichts tun, nur warten)
            break;

        case 4: // Schritt 4: LED 1 AUS
            LED_PIN_1_PORT->BSRR = LED_PIN_1_BIT_R;
            break;

        case 5: // Schritt 5: LED 2 AUS
            LED_PIN_2_PORT->BSRR = LED_PIN_2_BIT_R;
            break;

        case 6: // Schritt 6: LED 3 AUS
            LED_PIN_3_PORT->BSRR = LED_PIN_3_BIT_R;
            break;

        case 7: // Schritt 7: Pause (alle AUS)
            // (Nichts tun, nur warten)
            break;
    }

    // 5. "Fortschritt" weiterschalten
    state++;

    // Wenn wir beim letzten Schritt (7) waren, fange bei 0 wieder an.
    if (state > 7) {
        state = 0;
    }
}


/**
 * @brief Initialisiert ALLE unsere GPIOs
 */
void GPIO_Init_All(void) {

    // 1. Takte für Port A, B, C aktivieren
    RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN |
                     RCC_APB2ENR_IOPBEN |
                     RCC_APB2ENR_IOPCEN);

    /* --- LEDs (PC13, PA0, PA10, PB9) --- */

    // PC13 (Output)
    GPIOC->CRH = (GPIOC->CRH & ~GPIO_CRH_CNF13) | GPIO_CRH_MODE13_1;

    // PA0 (Output)
    GPIOA->CRL = (GPIOA->CRL & ~GPIO_CRL_CNF0) | GPIO_CRL_MODE0_1;

    // PA10 (Output)
    GPIOA->CRH = (GPIOA->CRH & ~GPIO_CRH_CNF10) | GPIO_CRH_MODE10_1;

    // PB9 (Output)
    GPIOB->CRH = (GPIOB->CRH & ~GPIO_CRH_CNF9) | GPIO_CRH_MODE9_1;

    /* --- Button (PB0) --- */

    // PB0 (Input Pull-Up)
    GPIOB->CRL = (GPIOB->CRL & ~GPIO_CRL_CNF0) | GPIO_CRL_CNF0_1;
    GPIOB->ODR |= GPIO_ODR_ODR0;
}
