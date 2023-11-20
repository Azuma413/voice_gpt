# このリポジトリについて
このリポジトリはGPT-4を用いてラズパイマウスを制御するデモを実装したROS2パッケージです。

Ubuntu22.04, ROS2 Humbleでの動作を前提としています。

# プログラムの概要
プログラムは全体としてROS2によって統合され，以下のようにノード同士が接続します。
![uml image](/image/voice_gpt.drawio.png)
## voice_gpt/chatrover_msgs
こちらのディレクトリは本パッケージで利用するROSのメッセージ型を定義するためのものです。

## voice_gpt/chat_rover
こちらのディレクトリは主にサービスサーバーの形式で実装されたROSノードを纏めたものです。

chat_rover/chat_rover/に各種ノードを実装したpythonファイルが格納されています。

chat_rover/launch/にlaunchファイルが格納されています。

## voice_gpt/chat_rover_bt
こちらのディレクトリは主にBehaviorTreeとそこで使われるROSノードを纏めたものです。

chat_rover_bt/srcにメインプログラムが格納されています。

chat_rover_bt/include/my_action_node.hppにBehaviorTreeのアクションノードが実装されています。

chat_rover_bt/include/my_ros_node.hppにBehaviorTreeで使われるROSのサービスクライアント等が実装されています。

chat_rover_bt/configにBehaviorTreeの構造を記したxmlファイルが格納されています。

## 処理の流れ
上の図のbt_nodeの内部は以下の図ようになっています。
![uml image](/image/chat_rover_bt.drawio.png)

BehaviorTree上で`VOSK`が呼び出されると，`vosk_node`のサービスが呼び出され，音声検出が始まります。音声を検出すると，その内容をBehaviorTreeへテキストとして返します。

`VOSK`は`GPT1`へそのテキストデータを送り，`GPT1`はそのデータをもとに`gpt1_node`を呼び出します。

`gpt1_node`は受け取ったテキストをGPT-4へ入力し，その出力を返します。GPT-4はテキストがロボットの命令であるかどうかを判断し，そうである場合は命令をより単純な形に分解します。

`GPT1`は`GPT2`へ受け取ったテキストデータを送ります。

`yolo_node`はリアルセンスから送られてくる映像データから物体を認識し，その物体名と座標をBehaviorTreeの`YOLO`へ送ります。`YOLO`はそのデータを`GPT2`へ送ります。

`GPT2`は`GPT1`から送られてきた単純な指示と，`YOLO`から送られてきた物体の位置情報を纏めて`gpt2_node`へ送ります。

`gpt2_node`は受け取ったデータをGPT-4へ入力し，その出力を返します。GPT-4はテキストデータから，ロボットの目標座標のリストを出力します。

`GPT2`は`gpt2_node`から値を受け取るとその情報を`SendPos`へ送ります。

`SendPos`は目標座標を`pos2vel_node`へ送ります。

`pos2vel_node`は目標座標と，`ar_node`から送られてくる現在位置のデータをもとに，ロボットの目標速度を定め，それをラズパイマウスへ送ります。

BehaviorTreeは以上の動作をTreeの構造に従い繰り返します。