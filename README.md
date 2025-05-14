## ビルド方法
### Setup submodule
```
git submodule update --init --recursive
```

### Build
```
cd extern/box2d
git checkout 9ebbbcd960ad424e03e5de6e66a40764c16f51bc
mkdir build
cd build
cmake -DBOX2D_BUILD_DOCS=OFF -DBOX2D_BUILD_UNIT_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX="./" ..
cmake --build . --config Release
cmake --build . --target install --config Release
cd ../../json
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX="./" ..
cmake --build .
cmake --build . --target install
cd ../../../src
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

## API一覧
以下は、C#側から呼び出すことができる主要なAPIとその概要です。

- **create_plugin**(digitalcurling3.StoneData* stone_data):

    - 機能: シミュレーターのインスタンスを作成します。

    - 引数: stone_data:     
    digitalcurling3::StoneData 型の配列へのポインタ。このバッファを通じて、シミュレーターとUnity間でストーンの位置情報をやり取りします。

    - 戻り値: SimulatorFCV1* 型のプラグインインスタンスへのポインタ。

    - 関連知識: シミュレーションを開始する前に必ず呼び出す必要があります。digitalcurling3::StoneData 構造体は、各ストーンの Vector2 型の position メンバ変数を持っています。

- **destroy_plugin**(SimulatorFCV1* plugin):

    - 機能: 作成したシミュレーターのインスタンスを破棄し、メモリを解放します。

    - 引数: plugin: 破棄する SimulatorFCV1 インスタンスへのポインタ。

    - 戻り値: なし。

    - 関連知識: シミュレーションが終了したら、忘れずに呼び出してリソースを解放してください。

- **reset_stones**(SimulatorFCV1* plugin):

    - 機能: 全てのストーンを初期位置にリセットし、非アクティブな状態にします。

    - 引数: plugin: 操作する SimulatorFCV1 インスタンスへのポインタ。

    - 戻り値: なし。

    - 内部処理: 内部的には SimulatorFCV1::reset_stones() が呼ばれます。ストーンの位置バッファ (stone_position_buffer) もリセットされます。

- **set_stones**(SimulatorFCV1* plugin):

    - 機能: Unity側で設定したストーンの位置情報をシミュレーターに反映させます。

    - 引数: plugin: 操作する SimulatorFCV1 インスタンスへのポインタ。

    - 戻り値: なし。

    - 内部処理: 内部的には SimulatorFCV1::set_stones() が呼ばれます。プラグイン作成時に渡された stone_position_buffer の情報に基づいて、Box2DのBody (stone_bodies) の位置とアクティブ状態を更新します。

- **set_velocity**(SimulatorFCV1* plugin,float velocity_x, float velocity_y, float angular_velocity, int total_shot, unsigned int shot_per_team, unsigned int team_id, unsigned int shot_status):

    - 機能: 投球するストーンの初速度、角速度、ゲームの状態（ショット数、チームID）を設定し、投球を開始します。

    - 引数: 
        - plugin: 操作する SimulatorFCV1 インスタンスへのポインタ。
        - velocity_x: ストーンのx方向の初速度
        - velocity_y: ストーンのy方向の初速度。
        - angular_velocity: ストーンの角速度（正: 反時計回り、負: 時計回り）。
        - total_shot: ゲーム全体でのショット数（0から）。
        - shot_per_team: 現在のチームが投球したショット数（0から7）。
        - team_id: 投球するチームのID（0: Team0、1: Team1）。
        - shot_status: このターンに投球するショットの種類を選択(0: draw_shot, 1: takeout_shot)
          デフォルトは0としてあります。(万博に使用するAIは全てdraw_shotを使用します)   

    - 戻り値: なし。

    - 内部処理: 内部的には SimulatorFCV1::set_velocity() が呼ばれます。指定されたストーンの速度、角速度を設定し、アクティブ状態にします。また、現在のゲームの状態（総ショット数、チーム内ショット数）を記録し、ファイブロックルールやノーティックルールの適用状況を初期化します。

- **set_status**(SimulatorFCV1* plugin, int status):

    - 機能: 現在のゲームの状態（適用するルール）を設定します。

    - 引数: 
        - plugin: 操作する SimulatorFCV1 インスタンスへのポインタ。
        - status: ゲームの状態を示す整数値（0: ファイブロックルール、1: ノーティックルール）。なお、この関数が呼ばれていないときはデフォルトでファイブロックルールが適用されます。

    - 戻り値: なし

    - 内部処理: 内部的には SimulatorFCV1::set_status() が呼ばれます

- **check_rule**(SimulatorFCV1* plugin):

    - 機能: シミュレーター内のストーンの最新の位置情報を、プラグイン作成時に渡されたバッファ (stone_position_buffer) に書き込みます

    - 引数: plugin: 操作する SimulatorFCV1 インスタンスへのポインタ。

    - 戻り値: なし。

    - 内部処理: 内部的には SimulatorFCV1::get_stones() が呼ばれます。必要に応じてファイブロックルールやノーティックルールに基づいたストーンの処理を行い、プレイエリア外に出たストーンを無効化し、最終的なストーンの位置を stone_position_buffer に格納します。

- **step**(SimulatorFCV1* plugin, int index, float coefficient):

    - 機能: シミュレーションを1タイムステップ進行させます。

    - 引数: 
        - plugin: 操作する SimulatorFCV1 インスタンスへのポインタ。
        - index: 操作するストーンのID（Team0: 0-7, Team1: 8-15）。通常は-1を指定することで、スイープしていない間は通常通りのシミュレーションを行います。
        - coefficient: 動摩擦係数に乗算する係数（通常は 1.0f）
        動摩擦係数の計算自体を変更することは難しかったため、求めた動摩擦係数に対して、さらに係数を掛けることでスイープを表現する。

    - 戻り値: 全てのストーンが停止した場合に false、それ以外の場合は true を返します。

    - 内部処理: 内部的には SimulatorFCV1::step() が呼ばれます。各awakeなストーンの速度、角速度を物理モデルに基づいて更新し、衝突判定と処理を行います。coefficient は特定のストーンの縦方向の加速度の計算に影響を与えます。

- Step関数内の簡単なまとめ

    - 物理モデル: Box2D物理エンジンをベースに、ストーンの運動（並進、回転）、氷との摩擦、ストーン同士の衝突をシミュレートしています 。

    - 縦方向の加速度: ストーンの速度に基づいて計算される、進行方向の減速です。速度が遅いほど減速が大きくなるような関数が使用されています。step 関数の coefficient 引数は、この加速度に影響を与えます。

    - ヨーレート: ストーンの速度と角速度に基づいて計算される、進行方向の変化率です。角速度の符号とストーンの速度によって左右どちらに曲がるかが決まります。

    - 角加速度: ストーンの速度に基づいて計算される、角速度の変化率です。速度が速いほど角速度の減少が緩やかになるような関数が使用されています。

    - 衝突: ストーン同士の衝突は、Box2Dの物理エンジンによって処理されます。ContactListener クラスで衝突後の処理 (PostSolve) が行われ、衝突時の法線方向と接線方向のインパルス（力積）が記録されます。反発係数は 1.0 に設定されています。

    - ストーンの状態: 各ストーンはアクティブ（awake）であるか非アクティブであるかの状態を持ちます。set_velocity で投球されたストーンはアクティブになり、速度がほぼ0になるか、プレイエリア外に出ると非アクティブになります。

- ゲームルール:

    - ファイブロックルール: 最初の5投までは、フリーガードゾーン（ティーライン手前、ハウス外）にある相手のストーンを直接テイクアウトすることができません。set_status(plugin, 0) でこのルールを適用します。freeguardzone_checker() と is_in_playarea() 関数が関連します。

    - ノーティックルール: 特定の条件（ティーライン手前、ハウスの前端より奥、センターライン上）を満たすストーンに、相手のストーンが接触した場合、接触した両方のストーンを元の位置に戻すルールです。set_status(plugin, 1) でこのルールを適用します。no_tick_checker() と no_tick_rule() 関数が関連します。

    - プレイエリア: ストーンの中心が定義された境界 (x_upper_limit, x_lower_limit, y_upper_limit, y_lower_limit) を超えると、プレイエリア外と判定され、無効化されます。


## Unityでのテスト
``` C#
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;

// ストーンの位置を格納するための構造体
[StructLayout(LayoutKind.Sequential)]
public struct Vector2
{
    public float x;
    public float y;
}

[StructLayout(LayoutKind.Sequential)]
public struct StoneData
{
    public Vector2 position;
}

public class CreateButton : MonoBehaviour
{
    [DllImport("simulator", CallingConvention = CallingConvention.Cdecl)]
    private static extern IntPtr create_plugin([Out] StoneData[] stone_data);
    [DllImport("simulator", CallingConvention = CallingConvention.Cdecl)]
    private static extern void destroy_plugin(IntPtr fcv1_simulator);
    [DllImport("simulator", CallingConvention = CallingConvention.Cdecl)]
    private static extern void reset_stones(IntPtr fcv1_simulator);
    [DllImport("simulator", CallingConvention = CallingConvention.Cdecl)]
    private static extern void set_stones(IntPtr fcv1_simulator);
    [DllImport("simulator", CallingConvention = CallingConvention.Cdecl)]
    private static extern void set_velocity(IntPtr fcv1_simulator, float velocity_x, float velocity_y, float angular_velocity, uint total_shot, uint shot_per_team, uint team_id, uint shot_status);
    [DllImport("simulator", CallingConvention = CallingConvention.Cdecl)]
    private static extern void set_status(IntPtr fcv1_simulator, int status);    // 0: five rock rule, 1: no tick rule
    [DllImport("simulator", CallingConvention = CallingConvention.Cdecl)]
    private static extern void check_rule(IntPtr fcv1_simualtor);
    [DllImport("simulator", CallingConvention = CallingConvention.Cdecl)]
    private static extern bool step(IntPtr fcv1_simulator, int stone_id, float coefficient);

    StoneData[] stone_data = new StoneData[16];
    private IntPtr fcv1_simulator; // fcv1_simulator をクラス内で宣言

    void Start()
    {
        Debug.Log("Creating plugin...");
        // "FCV1Simulator"を呼び出す
        fcv1_simulator = create_plugin(stone_data);
        Debug.Log("Plugin created");
        // 全てのストーンの位置をハック(0.0f, 0.0f)に移動
        reset_stones(fcv1_simulator);
        Debug.Log("Stones reset");
        // 適用するルール(ファイブロックかノーティックか)を選択
        set_status(fcv1_simulator, 0);   // 0: five rock rule, 1: no tick rule, Default is 0
        // 次のストーンの投球指示
        set_velocity(fcv1_simulator, velocity_x: 0.12f, velocity_y: 2.3f, angular_velocity: 1.57f, total_shot: 0, shot_per_team: 0, team_id: 0, shot_status: 0);
        Debug.Log("Velocity set");

        bool isRunning = true;
        int count = 0;
        // 投球指示をもとに、シミュレーションを開始(たまにバグって止まらないことがあるらしく、それらしきものを見つけて修正したが、一応シミュレーション回数の上限を50000と設けてシミュレーションを回す)
        while (isRunning && count < 50000)
        {
            isRunning = step(fcv1_simulator, stone_id: -1, coefficient: 1.0f);
            count++;
        }
        for (int i = 0; i < stone_data.Length; i++)
        {
            Debug.Log($"Stone {i}: Position = ({stone_data[i].position.x}, {stone_data[i].position.y})");
        }
        // 次のストーンの投球指示
        set_velocity(fcv1_simulator, velocity_x: 0.12f, velocity_y: 2.3f, angular_velocity: 1.57f, total_shot: 1, shot_per_team: 0, team_id: 1, shot_status: 1);
        isRunning = true;
        count = 0;
        while (isRunning && count < 50000)
        {
            isRunning = step(fcv1_simulator, stone_id: -1, coefficient: 1.0f);
            count++;
        }
        for (int i = 0; i < stone_data.Length; i++)
        {
            Debug.Log($"Stone {i}: Position = ({stone_data[i].position.x}, {stone_data[i].position.y})");
        }
        // プレイエリア外のストーン及び、ファイブロック・ノーティックルール適用後のストーンの位置を取得
        check_rule(fcv1_simulator);
        Debug.Log("Apply curling rule");
        for (int i = 0; i < stone_data.Length; i++)
        {
            Debug.Log($"Stone {i}: Position = ({stone_data[i].position.x}, {stone_data[i].position.y})");
        }
        // 呼び出した"fcv1_simulator"を削除
        destroy_plugin(fcv1_simulator);
    }


    // Update is called once per frame
    void Update()
    {
        
    }
}
```