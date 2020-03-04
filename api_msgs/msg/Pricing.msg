# Pricing Definitions of OANDA-API.
# Reference:
#    https://developer.oanda.com/rest-live-v20/pricing-df/
#    https://oanda-api-v20.readthedocs.io/en/latest/endpoints/pricing/pricingstream.html

# The date/time when the Price was created.
string time

# The Price’s Instrument.
string instrument

# The list of prices and liquidity available on the Instrument's bid side.
# It is possible for this list to be empty if there is no bid liquidity
# currently available for the Instrument in the Account.
PriceBucket[] bids

# The list of prices and liquidity available on the Instrument's ask side.
# It is possible for this list to be empty if there is no ask liquidity
# currently available for the Instrument in the Account.
PriceBucket[] asks

# The closeout bid Price. This Price is used when a bid is required to
# closeout a Position (margin closeout or manual) yet there is no bid
# liquidity. The closeout bid is never used to open a new position.
float32 closeout_bid

# The closeout ask Price. This Price is used when a ask is required to
# closeout a Position (margin closeout or manual) yet there is no ask
# liquidity. The closeout ask is never used to open a new position.
float32 closeout_ask

# Flag indicating if the Price is tradeable or not.
bool tradeable
