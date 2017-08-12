function Decoder(b, port) {

  // Amsterdam: 52.3731, 4.8924 = MSB 07FDD3 00BF1C, LSB D3FD07 1CBF00
  // La Paz: -16.4896, -68.1192 = MSB FD7BE0 F59B18, LSB E07BFD 189BF5
  // New York: 40.7127, -74.0059 = MSB 063657 F4B525, LSB 573606 25B5F4
  // Sidney: -33.8688, 151.2092 = MSB FAD500 17129C, LSB 00D5FA 9C1217

  // LSB, Least Significant Bit/Byte first! Your node likely sends MSB instead.

  // Sign-extend the 3rd and 6th bytes into a 4th and 8th byte:
  var lat = (b[0] | b[1]<<8 | b[2]<<16 | (b[2] & 0x80 ? 0xFF<<24 : 0)) / 10000;
  var lon = (b[3] | b[4]<<8 | b[5]<<16 | (b[5] & 0x80 ? 0xFF<<24 : 0)) / 10000;

  return {
    loc: {
      lat: lat,
      lon: lon
    },
    love: "TTN payload functions"
  };
}
