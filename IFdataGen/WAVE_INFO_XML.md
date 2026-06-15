# WAVE info.xml

`.wave` is a tar container. `info.xml` describes the waveform payload member and
the multi-channel VSG sample layout. Numeric units are fixed by this format:
frequency and sample-rate values are in hertz, duration is in seconds or
`hh:mm:ss.mmm`, sizes are in bytes, and counts are unitless.

## packed

`packed` stores all RF channels in one merged sample stream. All channels use
the same sample rate and quantization width. Each sample frame contains I/Q
data for every channel.

```xml
<?xml version='1.0' encoding='UTF-8'?>
<info version="2" type="mch" mode="packed">
  <WAVEFORM>waveform.dat</WAVEFORM>
  <sample_rate>62500000</sample_rate>
  <sample_count>11250000000</sample_count>
  <duration>00:03:00.000</duration>
  <quant_bits>3</quant_bits>
  <channel_count>3</channel_count>
  <sample_frame_bits>18</sample_frame_bits>
  <signal>GPS_L1CA GPS_L1C GPS_L2C GPS_L5</signal>

  <channel index="0">
    <center_frequency>1582244000</center_frequency>
    <bandwidth>40000000</bandwidth>
  </channel>

  <channel index="1">
    <center_frequency>1227600000</center_frequency>
    <bandwidth>40000000</bandwidth>
  </channel>

  <channel index="2">
    <center_frequency>1176450000</center_frequency>
    <bandwidth>40000000</bandwidth>
  </channel>
</info>
```

## channelized

`channelized` stores each RF channel as an independent stream. The payload may
interleave channel blocks, and each channel can use its own sample rate,
quantization width, and block size. Channel sample rates must have an integer
relationship that the generator and reader can handle.

```xml
<?xml version='1.0' encoding='UTF-8'?>
<info version="2" type="mch" mode="channelized">
  <WAVEFORM>waveform.dat</WAVEFORM>
  <sample_count>8100000000</sample_count>
  <duration>00:01:00.000</duration>
  <channel_count>3</channel_count>
  <signal>GPS_L1CA GPS_L1C GPS_L5 BDS_B1I BDS_B2A</signal>

  <channel index="0">
    <center_frequency>1582244000</center_frequency>
    <bandwidth>53500000</bandwidth>
    <sample_rate>54000000</sample_rate>
    <quant_bits>2</quant_bits>
    <block_size>54000</block_size>
  </channel>

  <channel index="1">
    <center_frequency>1223725000</center_frequency>
    <bandwidth>53500000</bandwidth>
    <sample_rate>54000000</sample_rate>
    <quant_bits>2</quant_bits>
    <block_size>54000</block_size>
  </channel>

  <channel index="2">
    <center_frequency>1176450000</center_frequency>
    <bandwidth>26500000</bandwidth>
    <sample_rate>27000000</sample_rate>
    <quant_bits>2</quant_bits>
    <block_size>27000</block_size>
  </channel>
</info>
```
