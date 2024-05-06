DEVICE=$1

case $DEVICE in

  dadbot)
    echo -n "192.168.8.249"
    ;;

  dadbot-meatball)
    echo -n "192.168.1.117"
    ;;

  dadbot-kgb)
    echo -n "192.168.234.182"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
