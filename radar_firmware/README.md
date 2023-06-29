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
Launch uniflash
```
~/ti/uniflash_8.2.0/node-webkit/nw ~/ti/uniflash_8.2.0
```

