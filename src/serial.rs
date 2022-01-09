//!
//! Asynchronous serial communication using UART/UART peripherals
//!
//! # Word length
//!
//! By default, the UART/UART uses 8 data bits. The `Serial`, `Rx`, and `Tx` structs implement
//! the embedded-hal read and write traits with `u8` as the word type.
//!
//! You can also configure the hardware to use 9 data bits with the `Config` `wordlength_9()`
//! function. After creating a `Serial` with this option, use the `with_u16_data()` function to
//! convert the `Serial<_, _, u8>` object into a `Serial<_, _, u16>` that can send and receive
//! `u16`s.
//!
//! In this mode, the `Serial<_, _, u16>`, `Rx<_, u16>`, and `Tx<_, u16>` structs instead implement
//! the embedded-hal read and write traits with `u16` as the word type. You can use these
//! implementations for 9-bit words.
//!

use core::fmt;
use core::marker::PhantomData;

use crate::rcc;
use embedded_hal::blocking;
use embedded_hal::prelude::*;
use embedded_hal::serial;
use nb::block;

use crate::gpio::{Const, SetAlternate};


use crate::gpio::gpio;

use crate::pac::UART0;

use crate::pac::UART1;

use crate::gpio::NoPin;

use crate::rcc::Clocks;

use crate::dma::traits::PeriAddress;

/// Serial error
#[non_exhaustive]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// Idle line state detected
    Idle,
}

pub mod config {
    use crate::time::Bps;
    use crate::time::U32Ext;

    pub enum WordLength {
        DataBits8,
        DataBits9,
    }

    pub enum Parity {
        ParityNone,
        ParityEven,
        ParityOdd,
    }

    pub enum StopBits {
        #[doc = "1 stop bit"]
        STOP1,
        #[doc = "0.5 stop bits"]
        STOP0P5,
        #[doc = "2 stop bits"]
        STOP2,
        #[doc = "1.5 stop bits"]
        STOP1P5,
    }

    // TODO: check how DMA is implemented if supported
    /*
    pub enum DmaConfig {
        None,
        Tx,
        Rx,
        TxRx,
    }
*/
    pub struct Config {
        pub baudrate: Bps,
        pub wordlength: WordLength,
        pub parity: Parity,
        pub stopbits: StopBits,
        //pub dma: DmaConfig,
    }

    impl Config {
        pub fn baudrate(mut self, baudrate: Bps) -> Self {
            self.baudrate = baudrate;
            self
        }

        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        pub fn wordlength_8(mut self) -> Self {
            self.wordlength = WordLength::DataBits8;
            self
        }

        pub fn wordlength_9(mut self) -> Self {
            self.wordlength = WordLength::DataBits9;
            self
        }

        pub fn stopbits(mut self, stopbits: StopBits) -> Self {
            self.stopbits = stopbits;
            self
        }
    }

    #[derive(Debug)]
    pub struct InvalidConfig;

    impl Default for Config {
        fn default() -> Config {
            let baudrate = 115_200_u32.bps();
            Config {
                baudrate,
                wordlength: WordLength::DataBits8,
                parity: Parity::ParityNone,
                stopbits: StopBits::STOP1,
              //  dma: DmaConfig::None,
            }
        }
    }
}

pub trait Pins<UART> {}
pub trait PinTx<UART> {
    type A;
}
pub trait PinRx<UART> {
    type A;
}
/*pub trait PinRst<UART> {
    type A;
}
pub trait PinClk<UART> {
    type A;
}
*/
/*pub trait PinIo<UART> {
    type A;
}
*/

impl<UART, TX, RX> Pins<UART> for (TX, RX)
where
    TX: PinTx<UART>,
    RX: PinRx<UART>,
{
}

/// A filler type for when the Tx pin is unnecessary
pub type NoTx = NoPin;
/// A filler type for when the Rx pin is unnecessary
pub type NoRx = NoPin;

impl<UART> PinTx<UART> for NoPin
where
    UART: Instance,
{
    type A = Const<0>;
}

impl<UART> PinRx<UART> for NoPin
where
    UART: Instance,
{
    type A = Const<0>;
}

macro_rules! pin {
    ($trait:ident<$UART:ident> for $gpio:ident::$PX:ident<$A:literal>) => {
        impl<MODE> $trait<$UART> for $gpio::$PX<MODE> {
            type A = Const<$A>;
        }
    };
}
//[AK]
//need to change according to our board
//apply pins to the PINS that support Rx and Tx
pin!(PinTx<UART1> for gpio0::p12_6<1>);
pin!(PinRx<UART1> for gpio0::p12_7<1>);


/// Serial abstraction
pub struct Serial<UART, PINS, WORD = u8> {
    UART: UART,
    pins: PINS,
    tx: Tx<UART, WORD>,
    rx: Rx<UART, WORD>,
}

/// Serial receiver
pub struct Rx<UART, WORD = u8> {
    _UART: PhantomData<UART>,
    _word: PhantomData<WORD>,
}

/// Serial transmitter
pub struct Tx<UART, WORD = u8> {
    _UART: PhantomData<UART>,
    _word: PhantomData<WORD>,
}

impl<UART, WORD> Rx<UART, WORD>
where
    UART: Instance,
{
    fn new() -> Self {
        Self {
            _UART: PhantomData,
            _word: PhantomData,
        }
    }
//[AK]
    /// Start listening for an rx not empty interrupt event
    ///
    /// Note, you will also have to enable the corresponding interrupt
    /// in the NVIC to start receiving events.
    pub fn listen(&mut self) {
        unsafe { (*UART::ptr()).cr1.modify(|_, w| w.rxneie().set_bit()) }
    }

    /// Stop listening for the rx not empty interrupt event
    pub fn unlisten(&mut self) {
        unsafe { (*UART::ptr()).cr1.modify(|_, w| w.rxneie().clear_bit()) }
    }

    /// Start listening for a line idle interrupt event
    ///
    /// Note, you will also have to enable the corresponding interrupt
    /// in the NVIC to start receiving events.
    pub fn listen_idle(&mut self) {
        unsafe { (*UART::ptr()).cr1.modify(|_, w| w.idleie().set_bit()) }
    }

    /// Stop listening for the line idle interrupt event
    pub fn unlisten_idle(&mut self) {
        unsafe { (*UART::ptr()).cr1.modify(|_, w| w.idleie().clear_bit()) }
    }

    /// Return true if the line idle status is set
    pub fn is_idle(&self) -> bool {
        unsafe { (*UART::ptr()).sr.read().idle().bit_is_set() }
    }

    /// Return true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        unsafe { (*UART::ptr()).sr.read().rxne().bit_is_set() }
    }

    /// Clear idle line interrupt flag
    pub fn clear_idle_interrupt(&self) {
        unsafe {
            let _ = (*UART::ptr()).sr.read();
            let _ = (*UART::ptr()).dr.read();
        }
    }
}

impl<UART, WORD> Tx<UART, WORD>
where
    UART: Instance,
{
    fn new() -> Self {
        Self {
            _UART: PhantomData,
            _word: PhantomData,
        }
    }

    /// Start listening for a tx empty interrupt event
    ///
    /// Note, you will also have to enable the corresponding interrupt
    /// in the NVIC to start receiving events.
    pub fn listen(&mut self) {
        unsafe { (*UART::ptr()).cr1.modify(|_, w| w.txeie().set_bit()) }
    }

    /// Stop listening for the tx empty interrupt event
    pub fn unlisten(&mut self) {
        unsafe { (*UART::ptr()).cr1.modify(|_, w| w.txeie().clear_bit()) }
    }

    /// Return true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        unsafe { (*UART::ptr()).sr.read().txe().bit_is_set() }
    }
}

impl<UART, PINS, WORD> AsRef<Tx<UART, WORD>> for Serial<UART, PINS, WORD> {
    fn as_ref(&self) -> &Tx<UART, WORD> {
        &self.tx
    }
}

impl<UART, PINS, WORD> AsRef<Rx<UART, WORD>> for Serial<UART, PINS, WORD> {
    fn as_ref(&self) -> &Rx<UART, WORD> {
        &self.rx
    }
}

impl<UART, PINS, WORD> AsMut<Tx<UART, WORD>> for Serial<UART, PINS, WORD> {
    fn as_mut(&mut self) -> &mut Tx<UART, WORD> {
        &mut self.tx
    }
}

impl<UART, PINS, WORD> AsMut<Rx<UART, WORD>> for Serial<UART, PINS, WORD> {
    fn as_mut(&mut self) -> &mut Rx<UART, WORD> {
        &mut self.rx
    }
}

impl<UART, TX, RX, WORD, const TXA: u8, const RXA: u8> Serial<UART, (TX, RX), WORD>
where
    TX: PinTx<UART, A = Const<TXA>> + SetAlternate<TXA>,
    RX: PinRx<UART, A = Const<RXA>> + SetAlternate<RXA>,
    UART: Instance,
{
    /*
        StopBits::STOP0P5 and StopBits::STOP1P5 aren't supported when using UART
        STOP_A::STOP1 and STOP_A::STOP2 will be used, respectively
    */
    pub fn new(
        UART: UART,
        mut pins: (TX, RX),
        config: config::Config,
        clocks: Clocks,
    ) -> Result<Self, config::InvalidConfig> {
        use self::config::*;

        unsafe {
            // NOTE(unsafe) this reference will only be used for atomic writes with no side effects.
            let rcc = &(*RCC::ptr());

            // Enable clock.
            UART::enable(rcc);
            UART::reset(rcc);
        }
        //[AK] calculate the baudrate
        let pclk_freq = UART::get_frequency(&clocks).0;
        let baud = config.baudrate.0;

        // The frequency to calculate UARTDIV is this:
        //
        // (Taken from STM32F411xC/E Reference Manual,
        // Section 19.3.4, Equation 1)
        //
        // 16 bit oversample: OVER8 = 0
        // 8 bit oversample:  OVER8 = 1
        //
        // UARTDIV =          (pclk)
        //            ------------------------
        //            8 x (2 - OVER8) x (baud)
        //
        // BUT, the UARTDIV has 4 "fractional" bits, which effectively
        // means that we need to "correct" the equation as follows:
        //
        // UARTDIV =      (pclk) * 16
        //            ------------------------
        //            8 x (2 - OVER8) x (baud)
        //
        // When OVER8 is enabled, we can only use the lowest three
        // fractional bits, so we'll need to shift those last four bits
        // right one bit

        // Calculate correct baudrate divisor on the fly
       /* let (over8, div) = if (pclk_freq / 16) >= baud {
            // We have the ability to oversample to 16 bits, take
            // advantage of it.
            //
            // We also add `baud / 2` to the `pclk_freq` to ensure
            // rounding of values to the closest scale, rather than the
            // floored behavior of normal integer division.
            let div = (pclk_freq + (baud / 2)) / baud;
            (false, div)
        } else if (pclk_freq / 8) >= baud {
            // We are close enough to pclk where we can only
            // oversample 8.
            //
            // See note above regarding `baud` and rounding.
            let div = ((pclk_freq * 2) + (baud / 2)) / baud;

            // Ensure the the fractional bits (only 3) are
            // right-aligned.
            let frac = div & 0xF;
            let div = (div & !0xF) | (frac >> 1);
            (true, div)
        } else {
            return Err(config::InvalidConfig);
        };
        */
        let tf = if baud == 9600 {
            0x174
        } else if baud == 115200 {
            0x1F
        } else {
            return Err(config::InvalidConfig);
        };
        unsafe { (*UART::ptr()).brr.write(|w| w.bits(tf)) };

        // Reset other registers to disable advanced UART features
        unsafe { (*UART::ptr()).cr2.reset() };
        unsafe { (*UART::ptr()).cr3.reset() };

        // Enable transmission and receiving
        // and configure frame
        unsafe {
            (*UART::ptr()).cr1.write(|w| {
                w.ue()
                    .set_bit()
                    .over8()
                    .bit(over8)
                    .te()
                    .set_bit()
                    .re()
                    .set_bit()
                    .m()
                    .bit(match config.wordlength {
                        WordLength::DataBits8 => false,
                        WordLength::DataBits9 => true,
                    })
                    .pce()
                    .bit(!matches!(config.parity, Parity::ParityNone))
                    .ps()
                    .bit(matches!(config.parity, Parity::ParityOdd))
            })
        };

       /* match config.dma {
            DmaConfig::Tx => unsafe { (*UART::ptr()).cr3.write(|w| w.dmat().enabled()) },
            DmaConfig::Rx => unsafe { (*UART::ptr()).cr3.write(|w| w.dmar().enabled()) },
            DmaConfig::TxRx => unsafe {
                (*UART::ptr())
                    .cr3
                    .write(|w| w.dmar().enabled().dmat().enabled())
            },
            DmaConfig::None => {}
        }
*/
        pins.0.set_alt_mode();
        pins.1.set_alt_mode();

        Ok(Serial {
            UART,
            pins,
            tx: Tx::new(),
            rx: Rx::new(),
        }
        .config_stop(config))
    }
    pub fn release(mut self) -> (UART, (TX, RX)) {
        self.pins.0.restore_mode();
        self.pins.1.restore_mode();

        (self.UART, self.pins)
    }
}

impl<UART, TX, WORD, const TXA: u8> Serial<UART, (TX, NoPin), WORD>
where
    TX: PinTx<UART, A = Const<TXA>> + SetAlternate<TXA>,
    UART: Instance,
{
    pub fn tx(
        UART: UART,
        tx_pin: TX,
        config: config::Config,
        clocks: Clocks,
    ) -> Result<Tx<UART, WORD>, config::InvalidConfig> {
        Self::new(UART, (tx_pin, NoPin), config, clocks).map(|s| s.split().0)
    }
}

impl<UART, RX, WORD, const RXA: u8> Serial<UART, (NoPin, RX), WORD>
where
    RX: PinRx<UART, A = Const<RXA>> + SetAlternate<RXA>,
    UART: Instance,
{
    pub fn rx(
        UART: UART,
        rx_pin: RX,
        config: config::Config,
        clocks: Clocks,
    ) -> Result<Rx<UART, WORD>, config::InvalidConfig> {
        Self::new(UART, (NoPin, rx_pin), config, clocks).map(|s| s.split().1)
    }
}

impl<UART, PINS, WORD> Serial<UART, PINS, WORD>
where
    UART: Instance,
{
    /// Starts listening for an interrupt event
    ///
    /// Note, you will also have to enable the corresponding interrupt
    /// in the NVIC to start receiving events.
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rxne => unsafe { (*UART::ptr()).cr1.modify(|_, w| w.rxneie().set_bit()) },
            Event::Txe => unsafe { (*UART::ptr()).cr1.modify(|_, w| w.txeie().set_bit()) },
            Event::Idle => unsafe { (*UART::ptr()).cr1.modify(|_, w| w.idleie().set_bit()) },
        }
    }

    /// Stop listening for an interrupt event
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rxne => unsafe { (*UART::ptr()).cr1.modify(|_, w| w.rxneie().clear_bit()) },
            Event::Txe => unsafe { (*UART::ptr()).cr1.modify(|_, w| w.txeie().clear_bit()) },
            Event::Idle => unsafe { (*UART::ptr()).cr1.modify(|_, w| w.idleie().clear_bit()) },
        }
    }

    /// Return true if the line idle status is set
    pub fn is_idle(&self) -> bool {
        unsafe { (*UART::ptr()).sr.read().idle().bit_is_set() }
    }

    /// Return true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        unsafe { (*UART::ptr()).sr.read().txe().bit_is_set() }
    }

    /// Return true if the tx register is empty (and can accept data)
    #[deprecated(since = "0.10.0")]
    pub fn is_txe(&self) -> bool {
        unsafe { (*UART::ptr()).sr.read().txe().bit_is_set() }
    }

    /// Return true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        unsafe { (*UART::ptr()).sr.read().rxne().bit_is_set() }
    }

    /// Return true if the rx register is not empty (and can be read)
    #[deprecated(since = "0.10.0")]
    pub fn is_rxne(&self) -> bool {
        unsafe { (*UART::ptr()).sr.read().rxne().bit_is_set() }
    }

    /// Clear idle line interrupt flag
    pub fn clear_idle_interrupt(&self) {
        unsafe {
            let _ = (*UART::ptr()).sr.read();
            let _ = (*UART::ptr()).dr.read();
        }
    }

    pub fn split(self) -> (Tx<UART, WORD>, Rx<UART, WORD>) {
        (self.tx, self.rx)
    }
}

impl<UART, PINS> Serial<UART, PINS, u8>
where
    UART: Instance,
{
    /// Converts this Serial into a version that can read and write `u16` values instead of `u8`s
    ///
    /// This can be used with a word length of 9 bits.
    pub fn with_u16_data(self) -> Serial<UART, PINS, u16> {
        Serial {
            UART: self.UART,
            pins: self.pins,
            tx: Tx::new(),
            rx: Rx::new(),
        }
    }
}

impl<UART, PINS> Serial<UART, PINS, u16>
where
    UART: Instance,
{
    /// Converts this Serial into a version that can read and write `u8` values instead of `u16`s
    ///
    /// This can be used with a word length of 8 bits.
    pub fn with_u8_data(self) -> Serial<UART, PINS, u8> {
        Serial {
            UART: self.UART,
            pins: self.pins,
            tx: Tx::new(),
            rx: Rx::new(),
        }
    }
}

impl<UART, PINS, WORD> serial::Read<WORD> for Serial<UART, PINS, WORD>
where
    UART: Instance,
    Rx<UART, WORD>: serial::Read<WORD, Error = Error>,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<WORD, Error> {
        self.rx.read()
    }
}

impl<UART> serial::Read<u8> for Rx<UART, u8>
where
    UART: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        // Delegate to the Read<u16> implementation, then truncate to 8 bits
        Rx::<UART, u16>::new().read().map(|word16| word16 as u8)
    }
}

/// Reads 9-bit words from the UART/UART
///
/// If the UART/UART was configured with `WordLength::DataBits9`, the returned value will contain
/// 9 received data bits and all other bits set to zero. Otherwise, the returned value will contain
/// 8 received data bits and all other bits set to zero.
impl<UART> serial::Read<u16> for Rx<UART, u16>
where
    UART: Instance,
{
    type Error = Error;

    fn read(&mut self) -> nb::Result<u16, Error> {
        // NOTE(unsafe) atomic read with no side effects
        let sr = unsafe { (*UART::ptr()).sr.read() };

        // Any error requires the dr to be read to clear
        if sr.pe().bit_is_set()
            || sr.fe().bit_is_set()
            || sr.nf().bit_is_set()
            || sr.ore().bit_is_set()
        {
            unsafe { (*UART::ptr()).dr.read() };
        }

        Err(if sr.pe().bit_is_set() {
            Error::Parity.into()
        } else if sr.fe().bit_is_set() {
            Error::Framing.into()
        } else if sr.nf().bit_is_set() {
            Error::Noise.into()
        } else if sr.ore().bit_is_set() {
            Error::Overrun.into()
        } else if sr.rxne().bit_is_set() {
            // NOTE(unsafe) atomic read from stateless register
            return Ok(unsafe { &*UART::ptr() }.dr.read().dr().bits());
        } else {
            nb::Error::WouldBlock
        })
    }
}

unsafe impl<UART> PeriAddress for Rx<UART, u8>
where
    UART: Instance,
{
    #[inline(always)]
    fn address(&self) -> u32 {
        &(unsafe { &(*UART::ptr()) }.dr) as *const _ as u32
    }

    type MemSize = u8;
}

impl<UART, PINS, WORD> serial::Write<WORD> for Serial<UART, PINS, WORD>
where
    UART: Instance,
    Tx<UART, WORD>: serial::Write<WORD, Error = Error>,
{
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        self.tx.flush()
    }

    fn write(&mut self, byte: WORD) -> nb::Result<(), Self::Error> {
        self.tx.write(byte)
    }
}

unsafe impl<UART> PeriAddress for Tx<UART, u8>
where
    UART: Instance,
{
    #[inline(always)]
    fn address(&self) -> u32 {
        &(unsafe { &(*UART::ptr()) }.dr) as *const _ as u32
    }

    type MemSize = u8;
}

impl<UART> serial::Write<u8> for Tx<UART, u8>
where
    UART: Instance,
{
    type Error = Error;

    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        // Delegate to u16 version
        Tx::<UART, u16>::new().write(u16::from(word))
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // Delegate to u16 version
        Tx::<UART, u16>::new().flush()
    }
}

/// Writes 9-bit words to the UART/UART
///
/// If the UART/UART was configured with `WordLength::DataBits9`, the 9 least significant bits will
/// be transmitted and the other 7 bits will be ignored. Otherwise, the 8 least significant bits
/// will be transmitted and the other 8 bits will be ignored.
impl<UART> serial::Write<u16> for Tx<UART, u16>
where
    UART: Instance,
{
    type Error = Error;

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        let sr = unsafe { (*UART::ptr()).sr.read() };

        if sr.tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn write(&mut self, word: u16) -> nb::Result<(), Self::Error> {
        // NOTE(unsafe) atomic read with no side effects
        let sr = unsafe { (*UART::ptr()).sr.read() };

        if sr.txe().bit_is_set() {
            // NOTE(unsafe) atomic write to stateless register
            unsafe { &*UART::ptr() }.dr.write(|w| w.dr().bits(word));
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<UART> blocking::serial::Write<u16> for Tx<UART, u16>
where
    UART: Instance,
{
    type Error = Error;

    fn bwrite_all(&mut self, buffer: &[u16]) -> Result<(), Self::Error> {
        for &b in buffer {
            loop {
                match self.write(b) {
                    Err(nb::Error::WouldBlock) => continue,
                    Err(nb::Error::Other(err)) => return Err(err),
                    Ok(()) => break,
                }
            }
        }
        Ok(())
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        loop {
            match <Self as serial::Write<u16>>::flush(self) {
                Ok(()) => return Ok(()),
                Err(nb::Error::WouldBlock) => continue,
                Err(nb::Error::Other(err)) => return Err(err),
            }
        }
    }
}

impl<UART> blocking::serial::Write<u8> for Tx<UART, u8>
where
    UART: Instance,
{
    type Error = Error;

    fn bwrite_all(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        for &b in bytes {
            loop {
                match self.write(b) {
                    Err(nb::Error::WouldBlock) => continue,
                    Err(nb::Error::Other(err)) => return Err(err),
                    Ok(()) => break,
                }
            }
        }
        Ok(())
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        loop {
            match <Self as serial::Write<u8>>::flush(self) {
                Ok(()) => return Ok(()),
                Err(nb::Error::WouldBlock) => continue,
                Err(nb::Error::Other(err)) => return Err(err),
            }
        }
    }
}

impl<UART, PINS> blocking::serial::Write<u16> for Serial<UART, PINS, u16>
where
    UART: Instance,
{
    type Error = Error;

    fn bwrite_all(&mut self, bytes: &[u16]) -> Result<(), Self::Error> {
        self.tx.bwrite_all(bytes)
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.tx.bflush()
    }
}

impl<UART, PINS> blocking::serial::Write<u8> for Serial<UART, PINS, u8>
where
    UART: Instance,
{
    type Error = Error;

    fn bwrite_all(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.tx.bwrite_all(bytes)
    }

    fn bflush(&mut self) -> Result<(), Self::Error> {
        self.tx.bflush()
    }
}

impl<UART, PINS, WORD> Serial<UART, PINS, WORD>
where
    UART: Instance,
{
    fn config_stop(self, config: config::Config) -> Self {
        self.UART.set_stopbits(config.stopbits);
        self
    }
}


use crate::pac::UART1 as uart_base;

// Implemented by all UART instances
pub trait Instance: crate::Sealed + rcc::Enable + rcc::Reset + rcc::GetBusFreq {
    #[doc(hidden)]
    fn ptr() -> *const uart_base::RegisterBlock;
    #[doc(hidden)]
    fn set_stopbits(&self, bits: config::StopBits);
}

macro_rules! halUART {
    ($UARTX:ty: ($UARTX:ident)) => {
        impl Instance for $UARTX {
            fn ptr() -> *const uart_base::RegisterBlock {
                <$UARTX>::ptr() as *const _
            }

            fn set_stopbits(&self, bits: config::StopBits) {
                use crate::pac::UART1::cr2::STOP_A;
                use config::StopBits;

                self.cr2.write(|w| {
                    w.stop().variant(match bits {
                        StopBits::STOP0P5 => STOP_A::STOP0P5,
                        StopBits::STOP1 => STOP_A::STOP1,
                        StopBits::STOP1P5 => STOP_A::STOP1P5,
                        StopBits::STOP2 => STOP_A::STOP2,
                    })
                });
            }
        }

        impl<UART, TX, RX, const TXA: u8, const RXA: u8> Serial<UART, (TX, RX)>
        where
            TX: PinTx<UART, A = Const<TXA>> + SetAlternate<TXA>,
            RX: PinRx<UART, A = Const<RXA>> + SetAlternate<RXA>,
            UART: Instance,
        {
            #[deprecated(since = "0.10.0")]
            pub fn $UARTX(
                UART: UART,
                pins: (TX, RX),
                config: config::Config,
                clocks: Clocks,
            ) -> Result<Self, config::InvalidConfig> {
                Self::new(UART, pins, config, clocks)
            }
        }
    };
}

// TODO: fix stm32f413 UARTs

macro_rules! halUart {
    ($UARTX:ty: ($UARTX:ident)) => {
        impl Instance for $UARTX {
            fn ptr() -> *const uart_base::RegisterBlock {
                <$UARTX>::ptr() as *const _
            }

            fn set_stopbits(&self, bits: config::StopBits) {
                use crate::pac::uart4::cr2::STOP_A;
                use config::StopBits;

                self.cr2.write(|w| {
                    w.stop().variant(match bits {
                        StopBits::STOP0P5 => STOP_A::STOP1,
                        StopBits::STOP1 => STOP_A::STOP1,
                        StopBits::STOP1P5 => STOP_A::STOP2,
                        StopBits::STOP2 => STOP_A::STOP2,
                    })
                });
            }
        }

        impl<UART, TX, RX, const TXA: u8, const RXA: u8> Serial<UART, (TX, RX)>
        where
            TX: PinTx<UART, A = Const<TXA>> + SetAlternate<TXA>,
            RX: PinRx<UART, A = Const<RXA>> + SetAlternate<RXA>,
            UART: Instance,
        {
            #[deprecated(since = "0.10.0")]
            pub fn $UARTX(
                UART: UART,
                pins: (TX, RX),
                config: config::Config,
                clocks: Clocks,
            ) -> Result<Self, config::InvalidConfig> {
                Self::new(UART, pins, config, clocks)
            }
        }
    };
}

halUART! { UART1: (UART1) }


impl<UART, PINS> fmt::Write for Serial<UART, PINS>
where
    Tx<UART>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.tx.write_str(s)
    }
}

impl<UART> fmt::Write for Tx<UART>
where
    Tx<UART>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        s.bytes()
            .try_for_each(|c| block!(self.write(c)))
            .map_err(|_| fmt::Error)
    }
}
