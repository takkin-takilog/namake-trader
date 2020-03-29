# TakeProfitOrder/StopLossOrder Definitions.
# Reference:
#    https://developer.oanda.com/rest-live-v20/order-df/#TakeProfitOrder
#    https://developer.oanda.com/rest-live-v20/order-df/#StopLossOrder

# The price threshold specified for the StopLoss/TakeProfit Order.
# The associated Trade will be closed by a market price that is
# equal to or worse/better than this threshold.
float32 price

# The current state of the Order.
api_msgs/OrderState order_state_msg
