function Decoder(bytes, port) {
  console.log(bytes.length);

  /** AIR QUALITY **/
  var pm10_standard = bytes[0];
  var pm25_standard = bytes[1];
  var pm100_standard = bytes[2];
  var pm10_env = bytes[3];
  var pm25_env = bytes[4];
  var pm100_env = bytes[5];
  var pm03 = bytes[6] | (bytes[7] << 8);
  var pm05 = bytes[8] | (bytes[9] << 8);
  var pm10 = bytes[10] | (bytes[11] << 8);
  var pm25 = bytes[12] | (bytes[13] << 8);
  var pm50 = bytes[14] | (bytes[15] << 8);
  var pm100 = bytes[16] | (bytes[17] << 8);

  /** TEMPERATURE **/
  var temperature = (bytes[18] | (bytes[19] << 8)) / 100.0;

  /** PRESSURE **/
  var pressure = (bytes[20] | (bytes[21] << 8)) / 10.0;

  /** HUMIDITY **/
  var humidity = (bytes[22] | (bytes[23] << 8)) / 100;

  /****** VOLTAGE ******/
  var rawVolt = bytes[24] | (bytes[35] << 8);
  var voltage = ((rawVolt * 2 * 3.3) / 1024).toFixed(2);
  console.log(voltage);
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {
    airQuality: {
      standard: {
        10: pm10_standard,
        25: pm25_standard,
        100: pm100_standard,
      },
      environmental: {
        10: pm10_env,
        25: pm25_env,
        100: pm100_env,
      },
      particles: {
        0.3: pm03,
        0.5: pm05,
        "1.0": pm10,
        2.5: pm25,
        "5.0": pm50,
        "10.0": pm100,
      },
    },
    temperature: temperature,
    pressure: pressure,
    humidity: humidity,
    voltage: voltage,
  };

  return decoded;
}
