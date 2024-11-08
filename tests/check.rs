#[cfg(test)]
mod tests {
    use embedded_hal_mock as hal;

    use self::hal::eh1::{
        delay::NoopDelay,
        i2c::{Mock as I2cMock, Transaction},
    };
    use super::*;
    #[test]
    fn serial() {
        assert_eq!(1, 1);
        
    }

}