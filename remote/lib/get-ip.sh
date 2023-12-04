DEVICE=$1

case $DEVICE in

  dadbot)
    echo -n "192.168.0.85"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
