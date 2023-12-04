DEVICE=$1

case $DEVICE in

  dadbot)
    echo -n "mini_bot"
    ;;

  *)
    echo -n ""
    >&2 echo "error invalid device name: ${DEVICE}"
    ;;
esac
