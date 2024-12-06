board=xiao_ble
dts="boards/xiao.overlay"
conf="boards/xiao.conf"


if [ $# -lt 1 ]; then
    echo "Usage: $0 <board_name> "
    echo " "
    echo "Examples:  "
    echo "       $0 naya_left "
    echo "       $0 naya_right "
    echo "       $0 naya_dongle "
    echo "       $0 xiao "
    exit 1
fi

if [[ "$1" == "xiao" || "$1" == "naya_left" || "$1" == "naya_right" || "$1" == "naya_dongle" ]]; then
    dts="boards/$1.overlay"
    conf="boards/$1.conf" 
    west build -b $board --  -DDTC_OVERLAY_FILE=$dts -DEXTRA_CONF_FILE=$conf

else
    echo "Invalid board Name"
    echo "Examples:  "
    echo "       $0 naya_left "
    echo "       $0 naya_right "
    echo "       $0 naya_dongle "
    echo "       $0 xiao "
    exit 1
fi


