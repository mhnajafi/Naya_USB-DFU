board=xiao_ble
dts="boards/xiao_ble.overlay"
conf="boards/xiao_ble.conf"


if [ $# -lt 1 ]; then
    echo "Usage: $0 <board_name> "
    echo " "
    echo "Examples:  "
    echo "       $0 naya_left "
    echo "       $0 naya_right "
    echo "       $0 naya_dongle "
    echo "       $0 xiao_ble "
    exit 1
fi

if [[ "$1" == "xiao_ble" || "$1" == "naya_left" || "$1" == "naya_right" || "$1" == "naya_dongle" ]]; then
    board=$1
    # dts="boards/$1.overlay"
    conf="boards/$1.conf"

    west build --pristine=auto -b $board -- -DEXTRA_CONF_FILE=$conf 

else
    echo "Invalid board Name"
    echo "Examples:  "
    echo "       $0 naya_left "
    echo "       $0 naya_right "
    echo "       $0 naya_dongle "
    echo "       $0 xiao_ble "
    exit 1
fi


