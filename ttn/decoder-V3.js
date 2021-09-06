function decodeUplink(input) {
      var data = {};

    data.latitude = ((input.bytes[0]<<16)>>>0) + ((input.bytes[1]<<8)>>>0) + input.bytesytes[2];
    data.latitude = (data.latitude / 16777215.0 * 180) - 90;
  
    data.longitude = ((input.bytes[3]<<16)>>>0) + ((input.bytes[4]<<8)>>>0) + input.bytes[5];
    data.longitude = (data.longitude / 16777215.0 * 360) - 180;
  
    var altValue = ((input.bytes[6]<<8)>>>0) + input.bytes[7];
    var sign = input.bytes[6] & (1 << 7);
    if(sign)
    {
        data.altitude = 0xFFFF0000 | altValue;
    }
    else
    {
        data.altitude = altValue;
    }
  
    data.hdop = input.bytes[8] / 10.0;

    return {
        data: data,
        warnings: [],
        errors: []
    };
}
