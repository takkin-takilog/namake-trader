# API fail reason code Definitions.
# Reference:
#    https://developer.oanda.com/rest-live-v20/transaction-df/#OrderCancelReason


# Order fail reason code definition
int32 REASON_UNSET=0
#  Filling the Order wasn't possible because the Order's instrument was halted.
int32 REASON_MARKET_HALTED=1
#  Connection error.
int32 REASON_CONNECTION_ERROR=1000
#  Others.
int32 REASON_OTHERS=9999


# The fail reason code of Topic and Service failed.
int32 reason_code
