# ETロボコン SPIKE用 高度ライントレースプログラム
##  概要

本プロジェクトは、LEGO SPIKE PrimeをプラットフォームとしたETロボコン向けのC++プログラムである。
PID制御（本実装ではPI制御）を基軸とし、黒・青ラインを高速かつ安定してトレースすることを目的とする。

最大の特徴は、周回数に応じて走行戦略を動的に変更する点にある。具体的には、走行速度、トレースするラインのエッジ（右／左）を切り替えることで、L/R両コースに対応可能な設計となっている。

注意: 現在のバージョンでは、障害物回避機能は/* */でコメントアウトされており、無効化されている。


## 制御ロジック

本プログラムの制御は、main_task内のwhile(1)ループによって実現される。

このループは `CONTROL_INTERVAL` (これは10ms)ごとに1サイクル実行され、以下の処理を高速で繰り返す。

### メインループ処理フロー

#### 1. パラメータ選択 (Parameter Selection)

現在の状態(currentState)に基づき、黒ライン用 (KP_BLACK等) または青ライン用 (KP_BLUE等) のPIDパラメータ一式を選択する。

#### 2. PID計算 (PID Calculation)

reflection = colorSensor.getReflection(): カラーセンサーから現在の反射光量を取得。

error = target - reflection: 目標値との差分（誤差）を算出。

integral += ...: 誤差を時間で積分。過去の誤差の蓄積を示し、定常偏差を補正する。INTEGRAL_MAXで上限・下限を設け、ワインドアップを防止。

derivative = ...: 前回誤差との差分から誤差の時間変化率（微分）を算出。急な変化を抑制し、安定性を高める（現在はKD_BLACK=0のため実質無効）。

turn = ...: P, I, Dの各項を合算し、最終的な操舵量turnを算出。

#### 3. 色情報判定 (Color Determination)

colorSensor.getHSV(): 現在の色情報をHSV形式で取得。

is_blue_candidate: HSV値が青色の閾値範囲内にあるかを判定。

is_black: V値（明度）が黒色の閾値以下であるかを判定。

is_turning_sharply: turn値の絶対値が閾値を超えているか。急旋回中かどうかの判定。

is_blue: is_blue_candidateの結果を、急旋回中でないという条件でフィルタリング。これにより、カーブの縁での色誤認を防止する。

#### 4. 状態遷移 (State Transition)

currentState == TRACE_BLACK の場合:

blue_ignore_timerが0の場合のみ色判定を実行。

is_blueがSWITCH_CONFIRM_COUNT回連続でtrueになった場合、currentStateをTRACE_BLUEへ遷移させる。

currentState == TRACE_BLUE の場合:

「黒色であり、かつ青色ではない」状態がSWITCH_CONFIRM_COUNT回連続した場合、currentStateをTRACE_BLACKへ遷移させる。

復帰時アクション:

lap_count++: 周回数を1増加させる。

if (lap_count <= 3): 3周目まで、交差点を確実に抜けるためSTRAIGHT_DURATION_MS秒間直進する。

blue_ignore_timer: 復帰後、一定時間（2秒間）青色を無視し、同じラインへの再反応を防ぐ。

モーター出力決定 (Motor Power Calculation)

currentBasePower: lap_countが1〜3周目の間はSLOW_POWER、それ以外はBASE_POWERを基準速度とする。

trace_on_right_edge: コース戦略の核心部分。

lap_countが0または1の時: true (右エッジトレース)

lap_countが2以上の偶数の時: false (左エッジトレース)

lap_countが2以上の奇数の時: true (右エッジトレース)

これにより、0, 1(右) -> 2(左) -> 3(右) -> 4(左)... という走行パターンが実現される。

leftPower, rightPower: currentBasePowerとturn値を合成。trace_on_right_edgeの値に応じて、turn値を左右のモーターに加算または減算する。

setPower(): 最終的な出力値を-100から100の範囲に収め、モーターに指令を出す。

#### 5. ループ終端処理

previousError = error: 現在の誤差を保存し、次回の微分計算に備える。

clock.sleep(): CONTROL_INTERVALで指定された時間だけ待機し、制御周期を一定に保つ。


## パラメータ調整ガイド

本プログラムの性能は、パラメータの値に大きく依存する。調整は体系的に行うことを推奨する。

### 最優先で調整すべきパラメータ

`TARGET_BLACK (目標反射値)`

役割: ライントレースの基準点。この値がズレていると、全てのPID制御が正しく機能しない。

調整方法: 事前にコースの黒と白の反射値を測定し、その中間値を設定する。

`KP_BLACK (Pゲイン)`

役割: ラインへの追従性（反応の速さ）。

調整方法: KD_BLACKを0の状態で、直線で蛇行し始め、かつカーブを曲がりきれるギリギリの値を探す。高すぎると激しく蛇行し、低すぎるとカーブで膨らむ。

`KD_BLACK (Dゲイン)`

役割: 走行の安定性。Pゲインによる行き過ぎ（オーバーシュート）を抑制するダンパー役。

調整方法: KP_BLACKを固定した上で、蛇行が収まり、動きが滑らかになる値を探す。高すぎると動きが硬直し、曲がらなくなる。現在のコードでは0だが、安定走行にはDゲインの導入が極めて有効である。

### 微調整用パラメータ

`KI_BLACK (Iゲイン)`

役割: モーターの個体差などで発生する、持続的な片寄り（定常偏差）の補正。

調整方法: PD制御だけでは解消できない直線での片寄りが気になる場合のみ、0.01のような非常に小さい値から試す。上げすぎると制御が破綻し、大きな揺れの原因となる。

`BASE_POWER (基本速度)`

役割: 全体の速度。

調整方法: PIDゲインが安定した後に、コースアウトしない範囲で徐々に上げていく。

`色判定閾値 (HSV, V値)`

役割: 青・黒の判定精度。

調整方法: 照明環境に大きく依存するため、実際のコース上でセンサーの値を確認しながら範囲を決定する。

