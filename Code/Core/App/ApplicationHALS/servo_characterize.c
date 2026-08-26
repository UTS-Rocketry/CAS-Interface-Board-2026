
#ifdef SERVO_CHARACTERIZE

/* Read one line of digits from UART5 into buf (blocking). Returns length. */
int uart_read_line(char *buf, int maxlen) {
    int i = 0;
    while (i < maxlen - 1) {
        uint8_t ch;
        /* blocking single-byte read */
        if (HAL_UART_Receive(&huart5, &ch, 1, HAL_MAX_DELAY) != HAL_OK) {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            if (i > 0) break;   /* end of line (ignore leading newlines) */
            else continue;
        }
        if (ch >= '0' && ch <= '9') {
            buf[i++] = (char)ch;
            HAL_UART_Transmit(&huart5, &ch, 1, HAL_MAX_DELAY); /* echo */
        }
    }
    buf[i] = '\0';
    return i;
}

void servo_characterize(void)
{
    /* Make sure TIM3 is running and CH4 output is live. */
    servo_init();

    printf("\r\n=== SERVO CHARACTERIZE ===\r\n");
    printf("Type a pulse width in us (500-2500) and press Enter.\r\n");
    printf("Start at 1500 and step outward. Watch for buzz at the stops.\r\n");

    char line[8];
    while (1) {
        printf("\r\nus> ");
        int n = uart_read_line(line, sizeof(line));
        if (n == 0) continue;

        int us = atoi(line);

        /* hard safety bound to the servo's absolute electrical range */
        if (us < 500)  us = 500;
        if (us > 2500) us = 2500;

        /* write CCR4 directly to bypass the MIN/MAX clamp during discovery */
        TIM3->CCR4 = (uint16_t)us;

        printf("\r\n  -> commanded %d us", us);

        /* If/when the max limit switch is wired (PB4), report its state.
         * With a pull-down, HIGH = switch triggered = brakes at full deploy. */
        GPIO_PinState sw = HAL_GPIO_ReadPin(LImitSwitchAirbrakes_GPIO_Port,
                                            LImitSwitchAirbrakes_Pin);
        printf("   [maxLimitSwitch=%s]\r\n", (sw == GPIO_PIN_SET) ? "TRIGGERED" : "open");
    }
}

#endif