DEVICE=$1

case $DEVICE in

  dadbot)
    echo -n "192.168.8.249"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
