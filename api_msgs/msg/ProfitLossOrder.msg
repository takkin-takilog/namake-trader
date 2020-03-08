# TakeProfitOrder/StopLossOrder Definitions.
# Reference:
#    https://developer.oanda.com/rest-live-v20/order-df/#TakeProfitOrder
#    https://developer.oanda.com/rest-live-v20/order-df/#StopLossOrder

# Order type definition
int32 TYP_TAKE_PROFIT = 1
int32 TYP_STOP_LOSS = 2

# Order state definition
int32 STS_PENDING = 1
int32 STS_FILLED = 2
int32 STS_TRIGGERED = 3
int32 STS_CANCELLED = 4

# The type of the Order.
int32 type

# The price threshold specified for the StopLoss/TakeProfit Order.
# The associated Trade will be closed by a market price that is
# equal to or worse/better than this threshold.
float32 price

# The current state of the Order.
int32 state