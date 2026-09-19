# 桌緣外參校正 Debug 指南

`field_calib_node` 用桌子邊緣求相機外參（`map → camera_link`）。本文件說明怎麼看 debug 畫面、怎麼判斷問題出在哪。
流程與原理見 [FLOW.md](FLOW.md) 第 6 節。

---

## 1. 啟動

```bash
# container，~/vision_ws
colcon build --packages-select field_calib && source install/setup.bash
ros2 launch field_calib field_calib.launch.py
```

- 啟動後直接開一個 **`field_calib`** 視窗（overlay）。`debug.window:=false` 可關閉。
- 視窗內按鍵：

| 按鍵 | 動作 |
|---|---|
| `c` | 開始校正：收 30 幀取 median，band 80 → 40 → 20 → 12 px 依序顯示（每輪停 1 秒），最後停在結果畫面 |
| `l` | 回到 live 畫面 |

- 也可以用 service 觸發：`ros2 service call /field_calib_node/calibrate std_srvs/srv/Trigger`
- 校正結果印在 terminal，並存到 `tools/calib/out/ros/<時間>/`：
  - `cam_tf.yaml`（含各線段品質、深度平面檢查）
  - 每一輪的 `band_<i>_<band>px_{overlay,strips,residuals}.png`
  - `tools/calib/out/ros/cam_tf.yaml` 永遠是最近一次的結果
- 節點**不會**自己改 TF。確認結果後，重新啟動相機並帶入 terminal 印出的 `cam_tf.*:=...` 參數。

---

## 2. 兩種畫面

| 畫面 | 何時 | 用的位姿 | 用途 |
|---|---|---|---|
| **live**（標題 `live`） | 平常，每 1 秒更新 | 目前 TF（rs_launch 的 `cam_tf.*`） | 檢查**目前使用中的外參**對不對；相機被碰、桌子被推會立刻看出來 |
| **calib**（標題 `calib`） | 按 `c` 之後 | 每一輪優化前 / 後 | 看校正過程中每條邊抓到哪裡、哪些點被丟掉 |

live 模式對不上時，terminal 會出現：

```
table edges do not match the current extrinsic: inlier RMS 2.6 px, accepted 22% (...)
```

門檻：`live.warn_rms_px`（預設 1.5 px）、`live.warn_inlier_ratio`（預設 70%）。

---

## 3. overlay 視窗怎麼看

左邊是相機影像，右邊是資訊面板。

### 影像上的標記

| 標記 | 意義 |
|---|---|
| 半透明色帶 | 每條線段的搜尋範圍（±band px） |
| 彩色粗線 | 模型桌緣（本輪優化**後**的位姿） |
| 白線 | 完整桌子外框 + 接縫（含圓角區） |
| 灰線 | 模型桌緣（本輪優化**前**的位姿）；live 模式和白線重疊 |
| 紅／綠箭頭 | map 原點與 X（紅）、Y（綠）軸，各 0.2 m |
| ● 綠 | accepted：採用的邊緣點 |
| × 橘 | weak_gradient：梯度太弱（`edge.grad_thresh`） |
| × 紫 | low_contrast：桌面／地板亮度差不夠（`edge.contrast_thresh`），常見於白色物品壓在邊上 |
| × 藍 | at_band_edge：最強邊緣在搜尋範圍邊界上 → 真正的邊可能在範圍外，或抓到別的東西 |
| × 紅 | huber_outlier：有抓到邊，但離模型線太遠（≥ 3 × `lm.delta`），優化時被降權 |

### 右側面板

- **四個角落放大**（origin = 上桌右上角 = map 原點、upper tl、lower bl、lower br）：白線角落應該貼著桌子圓角的延長線交點。
- **pose moved this stage**：這一輪位姿改變多少。最後一輪應該只剩 1–2 mm、< 0.05°。
- **每條線段統計**：

| 欄位 | 意義 |
|---|---|
| acc/n | 採用點數 / 影像內的取樣點數 |
| weak / lowc / edge / outl | 各剔除原因的點數 |
| RMS | 採用點到模型線的距離 RMS（px） |
| mean | 平均偏移（px），**正值 = 偵測到的邊在模型外側（地板那側）** |

### 正常的樣子（最後一輪 band 12 px）

- 綠點連成一條線，貼在白桌與地板的交界上。
- 各線段 RMS 約 0.2–0.9 px，mean 在 ±0.7 px 內。
- 被遮住的地方（人、筆電、機器人、線材）是 × 或沒有點，這是正常的，不影響結果。

---

## 4. 常見症狀與處理

| 症狀 | 可能原因 | 怎麼確認 / 處理 |
|---|---|---|
| live 畫面整個模型框**歪斜、扭轉**，warning 一直出現 | 使用中的 `cam_tf.*` 輸入錯（例：roll 正負號相反） | 看目前 TF：`ps -eo args \| grep static_transform_publisher`，和 `cam_tf.yaml` 逐一比對（**特別注意負號**） |
| live 突然從正常變成對不上 | 相機被碰到、桌子被推動 | 按 `c` 重新校正，比較 `cam_tf` 變化 |
| 某條邊幾乎全是 × 或 acc 很少 | 被人或物品擋住 | 清開後再校正；長期擋住的邊用 `field.disabled_segments:=lower_right` 關掉 |
| 某條邊有一段綠點整段偏 1–3 px | 抓到桌腳、線材、陰影、桌邊反光 | 看該輪的 `strips.png`（見第 5 節）確認抓到什麼；清開該區或關掉該邊 |
| 第一輪（80 px）大量藍 × | 初值離真值太遠，或搜尋範圍內有更強的邊 | 先把 `cam_tf.*` 大致修對（至少正負號），或加大 `calib.bands` 第一個值 |
| 最後一輪 pose moved 還很大（> 5 mm） | 還沒收斂 | 多加一輪小 band，例如 `calib.bands: [80, 40, 20, 12, 8]` |
| 左右兩條邊 mean 都是負（或都是正） | 模型桌子比實際寬（或窄） | 量實際桌子尺寸，改 `src/field_calib/config/field.yaml` |
| 深度平面檢查的相機高度和桌緣解差 1–2% | D455 深度尺度誤差，**或桌子尺寸不對** | 用捲尺量桌子外框；尺寸對了高度差還在，才歸因於深度 |
| 每次校正 `cam_tf` 差幾 mm / 零點幾度 | x、roll、yaw 彼此會互相抵消，數字看起來差很多但對定位影響小 | 比較實際定位結果，不要只比 `cam_tf` 數字 |

---

## 5. 進一步 debug：strips / residuals（離線）

overlay 看不出來時，用每一輪存下的 PNG，或離線工具逐輪看：

```bash
python3 tools/calib/capture_frames.py --n 30 --out tools/calib/out/cap1.npz     # container，擷取
python3 tools/calib/field_edge_calib.py tools/calib/out/cap1.npz --show          # 逐輪顯示三種畫面
python3 tools/calib/field_edge_calib.py tools/calib/out/cap1.npz --disable lower_right
python3 tools/calib/field_edge_calib.py --selftest                              # 合成影像自我測試
```

- **strips**：每條線段沿模型線「拉直」，橫軸 = 沿線位置、縱軸 = 法線方向放大 4×，**往下 = 往地板**。
  1 px 偏差會變成 4 px，可以看出綠點是抓在桌緣上，還是抓到桌腳、線材、陰影。
- **residuals**：每條線段「殘差 vs 沿線位置」：

| 形狀 | 意義 |
|---|---|
| 整條平移 | 尺寸或位置不對 |
| 一條斜線 | 旋轉不對，或那條桌邊本身不直 |
| 彎曲 | 畸變參數或桌邊彎曲 |
| 局部跳一段 | 誤抓（遮擋物、桌腳、線材） |

---

## 6. 可調參數（`src/field_calib/config/param.yaml`）

優先順序：command line > param.yaml > launch 預設 > 節點預設。

| 參數 | 預設 | 說明 |
|---|---|---|
| `field.disabled_segments` | `''` | 不使用的線段，逗號分隔：`upper_top, upper_right, upper_left, lower_right, lower_left, lower_bottom, seam` |
| `edge.grad_thresh` | 6.0 | 最小梯度（灰階 / px） |
| `edge.contrast_thresh` | 20.0 | 桌面與地板最小亮度差 |
| `edge.blur` | 1.0 | 抓邊前的高斯模糊 sigma（px） |
| `lm.delta` | 1.5 | Huber 門檻（px）；≥ 3 倍視為 outlier |
| `calib.frames` | 30 | 校正時取 median 的幀數 |
| `calib.bands` | [80, 40, 20, 12] | 每一輪的搜尋範圍（px） |
| `calib.stage_delay` | 1.0 | 每一輪畫面停留秒數 |
| `live.band` / `live.period` | 12 / 1.0 | live 檢查的搜尋範圍與更新週期 |
| `debug.window` | true | 是否開 overlay 視窗 |

場地尺寸、角落排除距離、接縫、下桌 x 偏移在 `src/field_calib/config/field.yaml`。
