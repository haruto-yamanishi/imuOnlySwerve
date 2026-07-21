# imuOnlySwerve

## 目的

`defaultSwerve` を元にした、IMU のみで自己位置推定の姿勢成分を更新する比較用プロジェクトです。

IMU 単体ではフィールド上の x/y 位置を直接観測できません。そのため、このプロジェクトでは「自己位置推定のうち、角度だけを IMU で推定する」構成にしています。x/y は最後の値を保持し、移動量の推定にはホイールエンコーダーもカメラも使いません。

## 使用する情報

- 使用する: IMU yaw
- 使用しない: ホイールエンコーダーによる移動量
- 使用しない: カメラ pose
- 更新しない: IMU だけでは観測できない x/y 位置

## 実装箇所

主な実装は `src/main/java/frc/robot/subsystems/DriveSubsystem.java` です。

- `SwerveParser` で `src/main/deploy/swerve` の YAGSL 設定を読み込み、`SwerveDrive` を生成します。
- `periodic()` で `swerveDrive.getYaw()` を読みます。
- `estimatedPose` の translation は維持し、rotation だけを IMU yaw に置き換えます。
- ホイールエンコーダーから得られる `SwerveModulePosition[]` は、推定 pose の更新には使いません。

## 推定処理の流れ

1. 現在の `estimatedPose` から x/y を保持する。
2. IMU から `swerveDrive.getYaw()` を取得する。
3. `new Pose2d(estimatedPose.getTranslation(), swerveDrive.getYaw())` を作る。
4. 作成した pose を新しい `estimatedPose` として表示する。

## 重要な実装ポイント

- IMU は角度を観測できますが、フィールド上の x/y 位置は観測できません。
- 加速度を二重積分すれば理論上は位置を推定できますが、実機ではノイズやバイアスにより短時間で大きくドリフトします。
- そのため、この実装では IMU の役割を heading 推定に限定しています。
- ロボットを動かしても x/y は基本的に変わらず、角度だけが変化します。

## 操作と出力

- Xbox コントローラー左スティックで並進、右スティック X 軸で回転します。
- A ボタンで `estimatedPose` と YAGSL odometry を原点 `Pose2d.kZero` にリセットします。
- SmartDashboard / Shuffleboard に `ImuOnly/X`, `ImuOnly/Y`, `ImuOnly/HeadingDeg` を出力します。
- `Field2d` は `IMU Only Pose` という名前で表示されます。

## 研究で見るべき点

- 回転したとき、heading がどれだけ滑らかに追従するか。
- 長時間静止したとき、yaw がどの程度 drift するか。
- 並進移動しても x/y が更新されないことから、IMU 単体では 2D pose 全体を推定できないことを確認する。
- ホイールエンコーダーのみのプロジェクトと比較し、heading に関しては IMU の方が安定する場面があるかを見る。

## 限界

IMU のみでは、2D pose のうち rotation は観測できますが、translation は観測できません。したがって、このプロジェクトは完全な自己位置推定ではなく、「姿勢推定だけを IMU に限定した比較実験」として扱います。

## シミュレーション実験

このプロジェクトには、自己位置推定の精度を測るための `AccuracyExperimentCommand` を追加しています。Autonomous を開始すると、全プロジェクト共通の走行パターンを実行します。

走行パターンは次の順です。

1. 1秒停止して pose を原点に合わせる。
2. 2.5秒前進する。
3. 2.5秒横移動する。
4. 2秒その場旋回する。
5. 2.5秒斜め後退する。
6. 停止する。

シミュレーションでは YAGSL/MapleSim の `getSimulationDriveTrainPose()` を真値 pose として使い、`estimatedPose` との差を計算します。Dashboard には次の値が出ます。

- `imuOnlySwerve/Accuracy/TranslationErrorMeters`
- `imuOnlySwerve/Accuracy/HeadingErrorDegrees`
- `imuOnlySwerve/Accuracy/RmsTranslationErrorMeters`
- `imuOnlySwerve/Accuracy/RmsHeadingErrorDegrees`
- `imuOnlySwerve/Accuracy/MaxTranslationErrorMeters`
- `imuOnlySwerve/Accuracy/MaxHeadingErrorDegrees`

CSV も `imuOnlySwerve-pose-accuracy.csv` として保存されます。保存先は Dashboard の `imuOnlySwerve/Accuracy/CsvPath` に表示されます。

実行方法は次の通りです。

```bash
./gradlew simulateJava
```

シミュレーション Driver Station で Autonomous を有効にすると、測定が始まります。

## 実機実験の前提

実機でも同じ Autonomous を使います。最初はバンパー周囲を含めて 4m x 4m 程度の安全な場所を確保し、床にスタート位置と向きをテープで印を付けます。

このプロジェクトでは x/y を更新しないため、並進移動の translation error は大きく出ます。実機では主に `ImuOnly/HeadingDeg` を見て、走行後のロボットの実際の向きと比較します。IMU の drift を見る場合は、停止状態で数分間 `ImuOnly/HeadingDeg` を記録します。
