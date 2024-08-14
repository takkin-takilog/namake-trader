# NaMaKe Trader

**NaMaKe Trader（ナマケ・トレーダー）** は、完全自動売買システム（システムトレード）になります。  
本システムを使用することであなたのトレード・スタイルが下図のようになります。  

<div align="center">
  <img src="https://raw.githubusercontent.com/takkin-takilog/files/main/namake-trader/change-your-trading-style.png">
</div>

具体的に列挙するなら、
* パソコンの画面に何時間も張り付く必要がなくなる。
* あなたが眠っているときや仕事をしているときも、あなたに代わってトレードしてくれる。
* 怠けられる時間が生まれる。
<br/>

もし、テクニカル分析などでトレード・ルールを自分の中で決めているのなら、そのルール、**本当に人の手で実践する必要ありますか？**  
ルール通りにトレードするのなら、**全てコンピュータに実践させれば良くないですか？**  

そのような疑問から僕が開発したのが完全自動売買システムの **NaMaKe Trader（ナマケ・トレーダー）** になります。  
自分で考えた自動売買アルゴリズムをプログラムすることで簡単に自動売買が行えるようになります。  

本ソフトウェアは MIT ライセンスのもとで公開していますので、改変、再配布等はMIT Licenseの範囲内で自由に行ってくださいね！　　

<br/>

## 1. システム動作環境
* 動作OS： **Linux (Ubuntu 24.04 推奨)**
* ソフトウェアPF： **[ROS2](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html) ([Jazzy Jalisco](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html) 推奨)**

### 1.1 依存関係
* `Pandas`
* `transitions`
* `oandapyV20`

<br/>

## 2. パッケージ構成
NaMaKe Traderは下図のようなROS2パッケージで構成されています。

<div align="center">
  <img src="https://raw.githubusercontent.com/takkin-takilog/files/main/namake-trader/namake-trader-architecture-design.png">
</div>


### 2.1 trade_app Package
`trade_app`（トレード・アプリケーション）パッケージは、自分で考えたトレード・アルゴリズムを実装することで世界で１つだけのシステム・トレードを実現することができます。  
ローソク足情報の取得や、新規・決済注文はROSメッセージ通信で抽象化されているため、各社が提供するAPIの仕様違いを意識する必要はありません！  
トレード・アルゴリズムを考えて実装することだけに専念することができます。  

> **Note**  
> サンプル・アプリケーションとして、ボリンジャーバンドを使用したシステム・トレード(`app_bb.py`)のコードがありますので、実装する際の参考にしてください。  
> ただし、ガチでトレードしますので、本番口座でなくデモ口座で動かすようにしましょう。  
> （本番口座で動かして損失が出たとしても、僕は責任を負いません。）  

### 2.2 trade_manager Package
`trade_manager`（トレード・マネージャー）パッケージは、ローソク足情報の管理や提供、各種注文の発行受付やスケジュール管理機能を提供します。  
本パッケージはアプリケーションとAPIパッケージの中間に位置するため、ミドルウェア的な存在となります。

### 2.3 api_server Package
`api_server`（APIサーバー）パッケージは、各社が提供するREST APIの機能をROSメッセージ通信で抽象化したラッパー・パッケージです。  
ROSメッセージ通信で抽象化することで各社が提供するAPIの仕様違いを吸収しています。  
そのため、APIを他社に変更したい場合は本パッケージを差し替えるだけでAPIの移行が完了します。  

> **Note**  
> 現状は[OANDA社提供のAPI](https://developer.oanda.com/rest-live-v20/introduction/)しかパッケージ化していないため、使用するならOANDA一択となります。  

<br/>

## 3. 使ってみよう！
本ソースコードには、サンプル・アプリケーションとしてボリンジャーバンドを使用したシステム・トレード(`app_bb.py`)があります。  
ここでは上記のアプリケーションを実際に稼働させるまでの手順を説明していきます。  

### 3.1 証券口座の準備
本システムはOANDA社が提供するREST APIを使用します。  
REST APIを使用するにはOANDAの口座が必要になりますので、REST APIが使用可能なコースで口座を開設してください。  
口座開設後は以下２つをメモしておきます。（あとで本システムに設定します。）  
- アカウントID
- APIトークン

### 3.2 UbuntuへROS2をインストール
UbuntuとROS2のインストール方法は僕が書いている以下の記事を参考にしてください。
- [UbuntuにROS×Pythonの開発環境を導入しよう！](https://takilog.com/introduce-ubuntu-ros-python-environment/)


### 3.3 環境セットアップ

1. 依存パッケージをインストールします。
```bash
$ pip install --break-system-packages pandas transitions oandapyV20
```

2. ワークスペースを作成します。
```bash
$ mkdir -p ~/git/
$ cd ~/git
```

3. このリポジトリをクローンします。
```bash
$ git clone https://github.com/takkin-takilog/namake-trader.git
```

4. 下記のファイルを開いてOANDA口座のアカウントID、APIトークンを設定します。
```bash
$ gedit namake-trader/trade_bringup/launch/api_server_oanda_launch.py
```
```python
def generate_launch_description() -> LaunchDescription:

    # ---------- Your OANDA Account Number and Access Token ----------
    # Demo Account
    PRAC_ACCOUNT_NUMBER = "***-***-*******-***"
    PRAC_ACCESS_TOKEN = "*************************-*************************"
    # Live Account
    LIVE_ACCOUNT_NUMBER = "***-***-*******-***"
    LIVE_ACCESS_TOKEN = "*************************-*************************"
```

アカウントID、APIトークンはデモ口座用と本番口座用があります。  
今回はデモ口座で動かすので、デモ口座の設定は必須になります。  

* デモ口座用
  * `PRAC_ACCOUNT_NUMBER`：アカウントID
  * `PRAC_ACCESS_TOKEN`：APIトークン
* 本番口座用
  * `LIVE_ACCOUNT_NUMBER`：アカウントID
  * `LIVE_ACCESS_TOKEN`：APIトークン

<br/>

> **Note**  
> デモ口座、本番口座の切り替えは以下のファイルの`use_env_live`で設定します。  
> デフォルトはデモ口座となっています。  
> ```bash
> $ gedit namake-trader/trade_bringup/launch/bringup_launch.py
> ```
> ```python
> def generate_launch_description() -> LaunchDescription:
>
>   # Use Live account environment if true
>   # Use Demo account environment if false
>   use_env_live = False
> ```
> 
> * `use_env_live` 
>   * `True`：本番口座で稼働
>   * `False`：デモ口座で稼働

### 3.4 ビルド＆実行

1. ビルド  
下記コマンドでビルドします。
```bash
$ colcon build --symlink-install
```

2. ワークスペースの環境設定  
ワークスペース情報を読み込みます。  
下記コマンドは新しい端末を開く度に実行してください。  
```bash
$ source install/local_setup.bash
```

3. 実行  
下記コマンドで実行します。
```bash
$ ros2 launch trade_bringup bringup_launch.py
```

4. 確認  
正常に実行されているか確認しましょう！  
別端末を開いて下記コマンドを実行するとノードグラフが表示されます。
```bash
$ cd ~/git
$ source install/local_setup.bash
$ rqt_graph
```

下記のノードグラフと同じになっていれば正常に実行されています。  
※各パーツの配置は多少異なる可能性があります。  

<div align="center">
  <img src="https://raw.githubusercontent.com/takkin-takilog/files/main/namake-trader/namake-trader-node-graph.png">
</div>

<br/>

## 4. 免責事項
* 本システムは、ユーザー各自の責任においてご利用ください。  
* 本システムを利用することにより、ユーザーが損失を被ったとしても、使用責任と損失リスクは全てユーザーに帰属します。  
また、一切のクレーム・損失・費用及びコスト（弁護士費用を含む）に対して、システム提供者の補償義務はなく、直接的・間接的・特殊な・偶然の・懲罰的な・派生的な損失が起こる可能性を示唆されていたとしても、システム提供者は責任を持ちません。  
* 本ソフトウェアの内容に関しては万全を期しておりますが、その内容の正確性および安全性、ユーザーにとっての有用性を保証するものではありません。  
* 本ソフトウェアが出力する情報に基づいて被ったいかなる損害についても、システム提供者は一切の責任を負わないものとします。  
* 本ソフトウェアに不具合が存在した場合に、それを出来るだけ修正するよう努めるものとします。  
ただし、全ての不具合を修正すること、及びソフトウェアの修正を迅速に行うということについては、これを保証するものではありません。  
* 本ソフトウェアは現状有姿の内容・機能でユーザーに提供され、システム提供者はその使用または性能に関して保証するものではなく、瑕疵担保責任を負わないものとします。  
* システム提供者は、ユーザーと他のユーザーまたは第三者との間で生じた紛争には関知いたしません。  

