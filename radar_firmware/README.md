```
sudo dpkg --add-architecture i386
sudo apt install build-essential -y
sudo apt install mono-complete -y
```

```
cd ~/rio_ws/build/mmwave_sdk_catkin/mmwave_sdk-prefix/src/mmwave_sdk-build/mmwave_sdk_03_06_00_00-LTS/packages/scripts/unix
source ./setenv.sh
```

# Update firmware using uniflash
Launch [uniflash](https://dev.ti.com/tirex/explore/content/radar_toolbox_1_10_00_13/docs/software_guides/using_uniflash_with_mmwave.html)
```
~/ti/uniflash_8.2.0/node-webkit/nw ~/ti/uniflash_8.2.0
```

Select IWR1843AOP
![uniflash select iwr1843aop](./doc/uniflash_select_iwr1843aop.png)

Select demo firmware
![uniflash select aop demo firmware](./doc/uniflash_select_aop_demo_firmware.png)

Select device `/dev/ttyUSB0`
![uniflash select device](./doc/uniflash_select_device_ttyUSB0.png)

Load image
![uniflash load image](./doc/uniflash_load_image.png)

# Test firmware with [mmWave Demo Visualizer](https://dev.ti.com/mmWaveDemoVisualizer)
Reset device, set S3 low

![set jumpers usb](./doc/set_jumpers_usb.png)

Connect device in visualizer on bottom left
![visualizer connect device](./doc/visualizer_connect_device.png)

Send config
![visualizer send config](./doc/visualizer_send_config.png)

View plots
![visualizer view plots](./doc/visualizer_view_plots.png)
