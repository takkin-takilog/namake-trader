# api_server_oanda

`api_server_oanda`はOANDA社が提供する[REST API](https://developer.oanda.com/docs/jp/)の機能をROSメッセージ通信で抽象化したラッパー・パッケージです。

# Node

- [pricing_publisher Node](https://github.com/takkin-takilog/namake-trader/tree/develop/api_server_oanda#pricing_publisher-node)
- [api_server Node](https://github.com/takkin-takilog/namake-trader/tree/develop/api_server_oanda#api_server-node)

## pricing_publisher Node
[pricing_publisher](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_oanda/api_server_oanda/pricing_publisher.py)ノードは特定の通貨ペアに対するリアルタイム為替レートをトピック配信します。  
OANDA REST APIサーバーとのストリーミング接続がアクティブであるかを判別するための死活監視シグナルも定周期でトピック配信します。  

### Published Topics

- **pricing_### ([api_server_msgs/Pricing](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/msg/Pricing.msg))**

    特定の通貨ペアに対するリアルタイム為替レート。  
    Parametersの`use_instrument.###`で`true`が設定された通貨ペアのみ有効となります。  
    `###`で指定可能な通貨ペアは以下となります。
    - usdjpy, eurjpy, eurusd, gbpjpy, audjpy, nzdjpy, cadjpy, chfjpy

- **heart_beat ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))**

    死活監視シグナル。アクティブ時は５秒毎に送信されます。

### Parameters

- **use_env_live (bool, default=`false`)**

    `true`の場合は本番口座環境を使用し、`false`の場合はデモ口座環境を使用します。

- **env_practice.account_number (string, default=None)**

    デモ口座のアカウントID

- **env_practice.access_token (string, default=None)**

    デモ口座のAPIトークン

- **env_live.account_number (string, default=None)**

    本番口座のアカウントID

- **env_live.access_token (string, default=None)**

    本番口座のAPIトークン

- **connection_timeout (int, default=10)**

    API接続アイムアウト時間[秒]

- **use_instrument.### (bool, default=`false`)**  

    `true`の場合、指定通貨ペアのpublishが有効になります。  
  （`###`で指定可能な通貨ペアはPublished Topicsの`pricing_###`を参照。）

#### Example:
```
pricing_publisher:
  ros__parameters:
    use_env_live: false
    env_practice:
      account_number: "***-***-*******-***"	# Replace your account_number (Demo account)
      access_token: "********************************-********************************"	# Replace your access_token (Demo account)
    env_live:
      account_number: "***-***-*******-***"	# Replace your account_number (Live account)
      access_token: "********************************-********************************"	# Replace your access_token (Live account)
    use_instrument:
      usdjpy: true
      eurjpy: true
      eurusd: true
      gbpjpy: false
      audjpy: false
      nzdjpy: false
      cadjpy: false
      chfjpy: false
    connection_timeout: 10  # [sec]
```

## api_server Node
[api_server](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_oanda/api_server_oanda/api_server.py)ノードは新規注文の生成やキャンセル、決済注文のほか、ローソク足情報、口座情報、リアルタイム為替レートなどを取得するサービスを提供します。

### Services

- **order_create([api_server_msgs/OrderCreateSrv](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/srv/OrderCreateSrv.srv))**

    新規注文を生成します。  
    注文が約定した場合は注文の種類に応じて下記のIDが返されます。
    - 成り行き注文：トレードID
    - 指値/逆指値注文：注文ID

- **trade_details([api_server_msgs/TradeDetailsSrv](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/srv/TradeDetailsSrv.srv))**

    指定されたトレードIDに紐づくトレード情報詳細を取得します。  

- **trade_crcdo([api_server_msgs/TradeCRCDOSrv](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/srv/TradeCRCDOSrv.srv))**

    指定されたトレードIDに紐づくトレード内容を変更します。  

- **trade_close([api_server_msgs/TradeCloseSrv](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/srv/TradeCloseSrv.srv))**

    指定されたトレードIDに紐づくトレードを決済します。  
    決済が約定した場合は決済情報が返されます。

- **order_details([api_server_msgs/OrderDetailsSrv](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/srv/OrderDetailsSrv.srv))**

    指定された注文IDに紐づく注文情報詳細を取得します。  

- **order_cancel([api_server_msgs/OrderCancelSrv](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/srv/OrderCancelSrv.srv))**

    指定された注文IDに紐づく注文を取り消します。  
    注文が未約定状態でないと、取り消しは失敗します。

- **candles_query([api_server_msgs/CandlesQuerySrv](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/srv/CandlesQuerySrv.srv))**

    指定された通貨ペア、時間足のローソク足(OHLC)情報を取得します。

- **account_query([api_server_msgs/AccountQuerySrv](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/srv/AccountQuerySrv.srv))**

    口座情報を取得します。

- **pricing_query([api_server_msgs/PricingQuerySrv](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/srv/PricingQuerySrv.srv))**

    指定された通貨ペアのリアルタイム為替レートを取得します。

### Parameters
- **use_env_live (bool, default=`false`)**

    `true`の場合は本番口座環境を使用し、`false`の場合はデモ口座環境を使用します。

- **env_practice.account_number (string, default=None)**

    デモ口座のアカウントID

- **env_practice.access_token (string, default=None)**

    デモ口座のAPIトークン

- **env_live.account_number (string, default=None)**

    本番口座のアカウントID

- **env_live.access_token (string, default=None)**

    本番口座のAPIトークン

- **connection_timeout (int, default=10)**

    API接続アイムアウト時間[秒]

#### Example:
```
api_server:
  ros__parameters:
    use_env_live: false
    env_practice:
      account_number: "***-***-*******-***"	# Replace your account_number (Demo account)
      access_token: "********************************-********************************"	# Replace your access_token (Demo account)
    env_live:
      account_number: "***-***-*******-***"	# Replace your account_number (Live account)
      access_token: "********************************-********************************"	# Replace your access_token (Live account)
    connection_timeout: 10  # [sec]
```
