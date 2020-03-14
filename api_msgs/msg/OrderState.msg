# Order state Definitions.
# Reference:
#    https://developer.oanda.com/rest-live-v20/order-df/#OrderState

# Order current state definition.
int32 STS_PENDING = 1
int32 STS_FILLED = 2
int32 STS_TRIGGERED = 3
int32 STS_CANCELLED = 4

# The current state of the order.
int32 state
