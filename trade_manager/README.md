# trade_manager

`trade_manager`はアプリケーション・パッケージとAPIサーバー・パッケージの中間に位置するミドルウェア・パッケージとなります。

# Node

- [candles_store Node](https://github.com/takkin-takilog/namake-trader/tree/develop/trade_manager#candles_store-node)
- [order_scheduler Node](https://github.com/takkin-takilog/namake-trader/tree/develop/trade_manager#order_scheduler-node)

## candles_store Node
[candles_store](https://github.com/takkin-takilog/namake-trader/blob/develop/trade_manager/trade_manager/candles_store.py)ノードは過去のローソク足情報(OHLCなど)
を保持し、アプリケーションへローソク足情報を提供します。  

<br>

**■シーケンス図（ローソク足情報の取得）**  

<div align="center">
  <img src="https://raw.githubusercontent.com/takkin-takilog/files/main/namake-trader/trade-manager/candles-store-sd-get-candle-info.png">
</div>

<br>

**■状態遷移図**  
<div align="center">
  <img src="https://raw.githubusercontent.com/takkin-takilog/files/main/namake-trader/trade-manager/candles-store-smd.png">
</div>

<br>

**■状態遷移表**  
　状態遷移表は下記スプレッドシートを参照してください。  
- [状態遷移表（スプレッドシート）](https://docs.google.com/spreadsheets/d/1z01OFVMXeKQ3xmyUc8lcu_W2l9EG-_J_q18Wq4B6hqU/edit?usp=sharing)


### Published Topics

- **###_@@@_latest_candle ([trade_manager_msgs/LatestCandle](https://github.com/takkin-takilog/namake-trader/blob/develop/trade_manager_msgs/msg/LatestCandle.msg))**

    時間足の最新OHLC(Open, High, Low, Close)データ。  
    Parametersの`use_instrument.###`、`use_granularity.@@@`で`true`が設定された通貨ペア、時間足のみ有効となります。  
    有効の場合はトピック名の`###`、`@@@`には下記の名称が代入されます。  
      ・`###`：通貨ペア（例：usdjpy, eurusd）  
      ・`@@@`：時間足（例：m10, h1）
  
### Services

- **candles_by_datetime([trade_manager_msgs/CandlesByDatetimeSrv](https://github.com/takkin-takilog/namake-trader/blob/develop/trade_manager_msgs/srv/CandlesByDatetimeSrv.srv))**

    過去のOHLC情報を取得します。  
    取得したい期間は日付・時間で指定します。  

- **candles_by_length([trade_manager_msgs/CandlesByLengthSrv](https://github.com/takkin-takilog/namake-trader/blob/develop/trade_manager_msgs/srv/CandlesByLengthSrv.srv))**

    過去のOHLC情報を取得します。  
    最新日付・時間から`length`で指定した個数分のデータが取得されます。  

### Parameters

- **use_instrument.### (bool, default=`false`)**  

    `true`の場合、指定通貨ペアのOHLC情報保持・提供が有効になります。  
    `###`で指定可能な通貨ペアは以下となります。  
    - usdjpy, eurjpy, eurusd, gbpjpy, audjpy, nzdjpy, cadjpy, chfjpy
 
- **use_granularity.@@@ (bool, default=`false`)**

    `true`の場合、指定時間足のOHLC情報保持・提供が有効になります。  
    `@@@`で指定可能な時間足は以下となります。  
    - m1, m2, m3, m4, m5, m10, m15, m30, h1, h2, h3, h4, h6, h8, h12, d, w

- **data_length.@@@ (int)**

    各時間足のOHLC情報を保持するデータ数。  
    `@@@`に時間足を指定します。  
    指定可能な時間足は`use_granularity.@@@`を参照。  

- **next_updatetime_ofs_sec (int, default=6)**

  次回更新時間のオフセット量[秒]。  
  例えば１時間足の場合、OHLC情報が更新されるのは13:00:00, 14:00:00, 15:00:00・・・ですが、時間通りだとサーバー側のOHLC情報が更新完了していない可能性があるため、取得時間にオフセットを設ける必要があります。  
  6秒が指定された場合は取得・更新時間が13:00:**06**, 14:00:**06**, 15:00:**06**・・・となります。

- **retry_interval_min (int, default=1)**

  リトライ時の次回更新インターバル時間[秒]。  
  状態遷移表T3遷移後に、"`next_updatetime_ofs_sec` + `retry_interval_min`"経過した後にT5遷移します。

- **fail_interval_min (int, default=10)**
  
  更新失敗時の次回更新インターバル時間[秒]。  
  更新失敗で状態遷移表T2遷移後に、"`next_updatetime_ofs_sec` + `fail_interval_min`"経過した後にT1遷移します。

- **retry_count_max (int, default=30)**

  リトライ回数の上限値。リトライ回数がこの数値に達すると更新失敗となり、T2遷移します。

- **self_retry_count_max (int, default=2)**  

  状態遷移表T4の自己遷移リトライ回数の上限値。自己遷移リトライ回数がこの数値に達するとT3遷移します。

#### Example:
```
candles_store:
  ros__parameters:
    use_instrument:
      usdjpy: true
      eurjpy: true
      eurusd: true
      gbpjpy: false
      audjpy: false
      nzdjpy: false
      cadjpy: false
      chfjpy: false
    use_granularity:
      m1: true
      m2: false
      m3: false
      m4: false
      m5: false
      m10: false
      m15: false
      m30: false
      h1: true
      h2: false
      h3: false
      h4: false
      h6: false
      h8: false
      h12: false
      d: false
      w: false
    data_length:
      m1: 14400    # 10 Days (60 * 24 * 10)
      m2: 7200     # 10 Days (30 * 24 * 10)
      m3: 4800     # 10 Days (20 * 24 * 10)
      m4: 3600     # 10 Days (15 * 24 * 10)
      m5: 2880     # 10 Days (12 * 24 * 10)
      m10: 14400   # 100 Days (6 * 24 * 100)
      m15: 9600    # 100 Days (4 * 24 * 100)
      m30: 4800    # 100 Days (2 * 24 * 100)
      h1: 720      # 1 Months (24 * 30 * 1)
      h2: 360      # 1 Months (12 * 30 * 1)
      h3: 240      # 1 Months (8 * 30 * 1)
      h4: 180      # 1 Months (6 * 30 * 1)
      h6: 120      # 1 Months (4 * 30 * 1)
      h8: 270      # 3 Months (3 * 30 * 3)
      h12: 180     # 3 Months (2 * 30 * 3)
      d: 365       # 1 Year (365 * 1)
      w: 48        # 1 Year (4 * 12 * 1)
    next_updatetime_ofs_sec: 6	# sec
    retry_interval_min: 1 # min
    fail_interval_min: 10 # min
    retry_count_max: 30
    self_retry_count_max: 2
```

## order_scheduler Node
[order_scheduler](https://github.com/takkin-takilog/namake-trader/blob/develop/trade_manager/trade_manager/order_scheduler.py)ノードは新規・決算注文の受付け、状態を管理します。  
新規注文は**成行**、**指値**、**逆指値**から指定します。

<br>

新規注文を受けると`OrderTiket`インスタンスを生成し、トレード完了時に消滅します。  
`OrderTiket`内で注文の状態を管理していきます。  

<br>

成行注文の受付けからトレード完了までのシーケンスです。  
**■シーケンス図（成行注文）**  

<div align="center">
  <img src="https://raw.githubusercontent.com/takkin-takilog/files/main/namake-trader/trade-manager/order-scheduler-sd-market.png">
</div>

<br>

指値、逆指値注文の受付けからトレード完了までのシーケンスです。  
**■シーケンス図（指値、逆指値注文）**  

<div align="center">
  <img src="https://raw.githubusercontent.com/takkin-takilog/files/main/namake-trader/trade-manager/order-scheduler-sd-limit-stop.png">
</div>

<br>

**■シーケンス図（参照）**  


<div align="center">
  <img src="https://raw.githubusercontent.com/takkin-takilog/files/main/namake-trader/trade-manager/order-scheduler-sd-ref-entry.png">
</div>
<br>
<div align="center">
  <img src="https://raw.githubusercontent.com/takkin-takilog/files/main/namake-trader/trade-manager/order-scheduler-sd-ref-exit.png">
</div>

<br>

以下、`OrderTiket`の状態遷移図と状態遷移表です。  

**■状態遷移図（OrderTiket）**  
<div align="center">
  <img src="https://raw.githubusercontent.com/takkin-takilog/files/main/namake-trader/trade-manager/order-scheduler-order-ticket-smd.png">
</div>

<br>

**■状態遷移表（OrderTiket）**  
　状態遷移表は下記スプレッドシートを参照してください。  
- [状態遷移表（スプレッドシート）](https://docs.google.com/spreadsheets/d/1VXyLzHF3eNEcxNzuICV7jmlG1YROk8Kce_bJi5gwcCY/edit?usp=sharing)

### Subscribed Topics

- **pricing_### ([api_server_msgs/Pricing](https://github.com/takkin-takilog/namake-trader/blob/develop/api_server_msgs/msg/Pricing.msg))**
  
  特定の通貨ペアに対するリアルタイム為替レート。  
  Parametersの`use_instrument.###`で`true`が設定された通貨ペアのみ有効となります。  
  有効の場合はトピック名の`###`には下記の名称が代入されます。  
    ・`###`：通貨ペア（例：usdjpy, eurusd）
  
- **heart_beat ([std_msgs/String](https://docs.ros2.org/foxy/api/std_msgs/msg/String.html))**

  死活監視シグナル。アクティブ時は５秒毎に送信されます。

### Services

- **order_register([trade_manager_msgs/OrderRegisterSrv](https://github.com/takkin-takilog/namake-trader/blob/develop/trade_manager_msgs/srv/OrderRegisterSrv.srv))**

  新規注文を登録します。  
  注文種類は**成行**、**指値**、**逆指値**の指定が可能で、同時に決済条件も指定する必要があります。  
  決済条件は**利益確定価格**、**損切り価格**の設定が必須となります。（いわゆるOCO注文）  
  登録成功時は**登録ID**が返却されます。
  
  `units`（取引数量）に0（ゼロ）をセットすると口座残高から取引数量が自動で計算される「複利運用モード」でトレードします。  
  詳細は以下の記事を参照してください。   
  ・[【NaMaKe Trader 活用術】複利で運用する方法](https://takilog.com/systra-utilization-compound-interest/)

- **trade_close_request([trade_manager_msgs/TradeCloseRequestSrv](https://github.com/takkin-takilog/namake-trader/blob/develop/trade_manager_msgs/srv/TradeCloseRequestSrv.srv))**

  **登録ID**に紐づく成行決済注文を出します。
  

### Parameters

- **max_leverage (float, default=20)**

  運用中でのレバレッジ上限値。  
  FX会社が設定している最大レバレッジより、やや小さい値を設定することを推奨します。  

- **max_position_count (int, default=8)**

  同時にエントリーできる上限値。  
  エントリー数が上限に達しているときに新規注文を受けた場合、注文は無効となります。  

- **use_weekend_order_stop (bool, default=8)**

  `true`の場合、週末の新規注文受付が停止されます。  
  週末にポジションを持ち越ししたくない場合に使用します。  

- **weekend_order_stop_time (string, default="0:00:00")**

  週末の新規注文受付を停止する時間を"H:M:S"形式で指定します。  
  `use_weekend_order_stop`が`true`の場合のみ、有効になります。  

- **use_weekend_all_close (bool, default=`false`)**

  `true`の場合、週末にポジションを全決済します。  
  週末にポジションを持ち越ししたくない場合に使用します。  

- **weekend_all_close_time (string, default="0:00:00")**

  週末にポジションを全決済する時間を"H:M:S"形式で指定します。  
  `use_weekend_all_close`が`true`の場合のみ、有効になります。  

- **account_updatetime_sec (int, default=30)**

  口座情報を取得する時間[秒]。  
  例えば、30が設定された場合、以下の時間で口座情報を取得します。  
  ・00:00:**30**, 00:01:**30**, 00:02:**30**・・・（１時間毎に取得）  

- **poll_interval_min (int, default=1)**

  下記の状態遷移するインターバル時間[分]。  
  ・「新規約定待ち(S2)」→「新規約定状態確認中(S3)」（T4遷移条件）  
  ・「決済約定待ち(S5)」→「決済約定状態確認中(S6)」（T10遷移条件）  

#### Example:
```
order_scheduler:
  ros__parameters:
    max_leverage: 20.0
    max_position_count: 8
    use_weekend_order_stop: false
    weekend_order_stop_time: 0:00:00  # "%H:%M:%S"
    use_weekend_all_close: false
    weekend_all_close_time: 0:00:00  # "%H:%M:%S"
    account_updatetime_sec: 30	# sec
    poll_interval_min: 1	# min
```

