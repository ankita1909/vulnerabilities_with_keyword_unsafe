use psoc6_01_pac as psoc;
    
    pub fn init_uart_registers(p:psoc::Peripherals ) -> psoc::Peripherals
    {
    
    //let p=psoc::Peripherals::take().unwrap();
    //let _cp=cortex_m::Peripherals::take().unwrap();

    /* Initializing SCB5 registers needed for UART*/
    let ctrl=&p.SCB5.ctrl;
    let uart_ctrl=&p.SCB5.uart_ctrl;
    let uart_tx_ctrl=&p.SCB5.uart_tx_ctrl;
    let tx_ctrl = &p.SCB5.tx_ctrl;
    let tx_fifo_ctrl = &p.SCB5.tx_fifo_ctrl;
    let uart_flow_ctrl = &p.SCB5.uart_flow_ctrl;
    let uart_rx_ctrl = &p.SCB5.uart_rx_ctrl;
    let rx_ctrl = &p.SCB5.rx_ctrl;
    let rx_fifo_ctrl = &p.SCB5.rx_fifo_ctrl;
    let rx_match = &p.SCB5.rx_match;
    let intr_tx = &p.SCB5.intr_tx;
    let intr_tx_set = &p.SCB5.intr_tx_set;
    let tx_fifo_wr = &p.SCB5.tx_fifo_wr;
    let rx_fifo_rd = &p.SCB5.rx_fifo_rd;

    /*Initialising the GPIO Ports for UART and LED blinking */
    //let p13 = &p.GPIO.prt13;
    let p5 = &p.GPIO.prt5;
    let p5_h = &p.HSIOM.prt5;
    

    /*Configure UART ports */
    p5.cfg.write(|w| unsafe { w.drive_mode1().bits(6)
                                .in_en1().clear_bit()
                                .drive_mode0().bits(0)
                                .in_en0().set_bit()});
                                
    p5.out.write(|w| w.out1().set_bit()
                        .out0().set_bit());

    p5.out_inv.write(|w| w.out1().set_bit()
                        .out0().set_bit());

    p5.out_clr.write(|w| w.out1().set_bit()
                        .out0().set_bit());

    p5.out_set.write(|w| w.out1().set_bit()
                        .out0().set_bit());

    p5_h.port_sel0.write(|w| unsafe{w.io0_sel().bits(18)
                                        .io1_sel().bits(18)});



    /* Configure UART Interface Registers*/
    ctrl.write(|w| unsafe{w.ovs().bits(7)
                            .mode().bits(2)
                            .enabled().set_bit()
                            .byte_mode().set_bit()});
    

    
    uart_ctrl.write(|w| unsafe {w.mode().bits(0)});

    /* Tx Configuration */
    uart_tx_ctrl.write(|w| unsafe {w.stop_bits().bits(1)
                                    .retry_on_nack().clear_bit()
                                    .parity().clear_bit()
                                    .parity_enabled().clear_bit()});

    tx_ctrl.write(|w| unsafe{w.msb_first().clear_bit()
                        .open_drain().clear_bit()
                        .data_width().bits(7)}
                    );

    tx_fifo_ctrl.write(|w| unsafe { w.trigger_level().bits(63)});

    uart_flow_ctrl.write(|w| unsafe { w.cts_enabled().clear_bit()
                                        .cts_polarity().clear_bit()
                                        .rts_polarity().clear_bit()
                                        .trigger_level().bits(20)}
                                    );

    /* Rx Configuration */
    uart_rx_ctrl.write(|w| unsafe{w.stop_bits().bits(1) 
                                    .polarity().clear_bit()
                                    .parity().clear_bit()
                                    .parity_enabled().clear_bit()}
                                );
    rx_ctrl.write(|w| unsafe{w.msb_first().clear_bit()
                                .median().clear_bit()
                                .data_width().bits(7)}
                            );
    rx_fifo_ctrl.write(|w| unsafe{w.trigger_level().bits(0)});

    rx_match.write(|w| unsafe {w.addr().bits(0)
                                .mask().bits(0)});

    /* Interrupt Configuration */
    intr_tx_set.write(|w| unsafe{w.bits(83)});
    intr_tx.write(|w| unsafe{w.bits(83)});

    /*select clock as clk_peri */

    let peri_cmd = &p.PERI.div_cmd;
    let peri_div_8_ctl = &p.PERI.div_8_ctl[0];
    let peri_div_16_ctl = &p.PERI.div_16_ctl[0];
    let peri_clock_ctl = &p.PERI.clock_ctl[5];


    peri_cmd.write(|w| unsafe{w.enable().set_bit()
                                .pa_type_sel().bits(3)
                                .pa_div_sel().bits(255)
                                .type_sel().bits(0)
                                .div_sel().bits(0)});

    peri_cmd.write(|w| unsafe{w.enable().set_bit()
                                .pa_type_sel().bits(3)
                                .pa_div_sel().bits(255)
                                .type_sel().bits(1)
                                .div_sel().bits(0)});

    peri_div_8_ctl.write(|w| unsafe{w.bits(65281)
                                    });

    peri_div_16_ctl.write(|w|unsafe{w.bits(27649)});

    peri_clock_ctl.write(|w| unsafe{w.bits(256)});

    p

                                }