/** 
    Copyright (C) 2024 Alex Rau

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
*/

/**

 *  Preamble and sync words: no whitening, no manchester
 *  Payload: Manchester encoded

 - Preamble: aaaa
 - Sync Word: 0xDE 0xAD
 - Model ID: 8bit
 - Temp 1: 16bit
 - Temp 2: 16bit
 - Temp 3: 16bit
 - Temp 4: 16bit
 - VBatt: 16bit
 - CRC: 16bit (CCITT)

*/

#include "decoder.h"


#define TOTAL_PAYLOAD_LENGTH_WITH_CRC_BYTES 13

static int dyi_temperature_sensor_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{

    decoder_log(decoder, 1, __func__, "DIY Temperature Array Sensor");


    decoder_log_bitbuffer(decoder, 1, __func__, bitbuffer, "");

    // There will only be one row
    if (bitbuffer->num_rows > 1) {
        decoder_logf(decoder, 1, __func__, "Too many rows: %d", bitbuffer->num_rows);
        return DECODE_FAIL_SANITY;
    }

    // search for expected start sequence
    uint8_t const preambleSync[] = {0xaa, 0xaa, 0xde, 0xad}; // preamble + sync word (32 bits)
    unsigned int bit_offset      = bitbuffer_search(bitbuffer, 0, 0, preambleSync, sizeof(preambleSync) * 8);

    bit_offset += sizeof(preambleSync) * 8;

    decoder_logf(decoder, 1, __func__, "Bit Offset: %d", bit_offset);
    decoder_logf(decoder, 1, __func__, "Bits per row: %d", bitbuffer->bits_per_row[0]);

    if (bit_offset + TOTAL_PAYLOAD_LENGTH_WITH_CRC_BYTES * 2 * 8 > bitbuffer->bits_per_row[0]) {
        decoder_logf(decoder, 1, __func__, "Received payload too short...");
        return DECODE_ABORT_EARLY;
    }
    
/*    uint8_t msg_byte_count = (bitbuffer->bits_per_row[0] - bit_offset) / 8;

    if (msg_byte_count < 2) {
        decoder_logf(decoder, 1, __func__, "Packet too short: %d bytes", msg_byte_count);
        return DECODE_ABORT_LENGTH;
    }
    else if (msg_byte_count > 10) {
        decoder_logf(decoder, 1, __func__, "Packet too long: %d bytes", msg_byte_count);
        return DECODE_ABORT_LENGTH;
    }
    else {
        decoder_logf(decoder, 2, __func__, "packet length: %d", msg_byte_count);
    }*/

    bitbuffer_t databits = {0};

    bitbuffer_manchester_decode(bitbuffer, 0, bit_offset, &databits, 50* 8);
    bitbuffer_invert(&databits);

    decoder_log_bitbuffer(decoder, 1, __func__, &databits, "");
    decoder_logf(decoder, 1, __func__, "Bits per row (after manchester decode): %d", databits.bits_per_row[0]);

    if (databits.bits_per_row[0] < TOTAL_PAYLOAD_LENGTH_WITH_CRC_BYTES * 8) {
         decoder_log(decoder, 1, __func__, "manchester_decode fail. Result after decoding not correct length.");
        return DECODE_FAIL_SANITY;
    }

    uint8_t *b = databits.bb[0];

    data_t *data;
    int device, crc;
    float temp_raw_1, temp_raw_2, temp_raw_3, temp_raw_4, vBatt_raw;

    device  = (b[0]);
    temp_raw_1 = (int16_t)(((b[1]) << 8) | (b[2] )) / 100.0;
    temp_raw_2 = (int16_t)(((b[3]) << 8) | (b[4] )) / 100.0;
    temp_raw_3 = (int16_t)(((b[5]) << 8) | (b[6] )) / 100.0;
    temp_raw_4 = (int16_t)(((b[7]) << 8) | (b[8] )) / 100.0;
    vBatt_raw =  (int16_t)(((b[9]) << 8) | (b[10] )) / 100.0;
    crc =        (int16_t)(((b[11]) << 8) | (b[12] ));

    // uint16_t crc_calc = crc16(b, 14, 0x1021, 0x0000);

    decoder_logf(decoder, 1, __func__, "CRC: %d", crc);

    /* clang-format off */
    data = data_make(
            "model",            "",                DATA_STRING,    "DYI Temperature Array Sensor",
            "id",               "Id",              DATA_INT,       device,
            "temperature1_C",    "Temperature 1",  DATA_FORMAT,    "%.2fC",  DATA_DOUBLE,    temp_raw_1,
            "temperature2_C",    "Temperature 2",  DATA_FORMAT,    "%.2fC",  DATA_DOUBLE,    temp_raw_2,
            "temperature3_C",    "Temperature 3",  DATA_FORMAT,    "%.2fC",  DATA_DOUBLE,    temp_raw_3,
            "temperature4_C",    "Temperature 4",  DATA_FORMAT,    "%.2fC",  DATA_DOUBLE,    temp_raw_4,
            "vBatt",             "Battery Voltage",  DATA_FORMAT,    "%.2fV",  DATA_DOUBLE,    vBatt_raw,
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

static char const *const output_fields[] = {
        "model",
        "id",
        "temperature1_C",
        "temperature2_C",
        "temperature3_C",
        "temperature4_C",
        "vBatt",
        NULL,
};

r_device const diy_temperature_array = {
        .name        = "DIY Temperature Array Sensor",
        .modulation  = FSK_PULSE_PCM,
        .short_width = 104,
        .long_width  = 104,
        .reset_limit = 9600,
        .decode_fn   = &dyi_temperature_sensor_callback,
        .fields      = output_fields,
        .disabled    = 0,
};

