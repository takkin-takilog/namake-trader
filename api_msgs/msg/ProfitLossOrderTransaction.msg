# takeProfitOrderTransaction/StopLossOrderTransaction Definitions.
# Reference:
#    https://developer.oanda.com/rest-live-v20/transaction-df/#takeProfitOrderTransaction
#    https://developer.oanda.com/rest-live-v20/transaction-df/#StopLossOrderTransaction

# Profit/Loss order transaction definition
int32 TRA_TAKE_PROFIT_ORDER = 1
int32 TRA_STOP_LOSS_ORDER = 2

# The type of the Order Transaction.
int32 type

# The price threshold specified for the StopLoss/TakeProfit Order.
# The associated Trade will be closed by a market price that is
# equal to or worse/better than this threshold.
float32 price
