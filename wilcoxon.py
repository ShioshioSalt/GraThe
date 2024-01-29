import os
import pandas as pd
import scipy.stats as stats

# フォルダのパス
folder_path = 'result(CSV)' #ここにCSVファイルがあるフォルダのパスを入れる

# フォルダ内のCSVファイルのリストを取得
csv_files = [f for f in os.listdir(folder_path) if f.endswith('.csv')]

# 2つのCSVファイルを選択
file_path1 = os.path.join(folder_path, csv_files[0])
file_path2 = os.path.join(folder_path, csv_files[1])

# データの読み込み
column_names = ['duration', 'distance']
data1 = pd.read_csv(file_path1, names=column_names)
data2 = pd.read_csv(file_path2, names=column_names)

# データを数値型に変換し、NaN値を除外する
data1 = data1.apply(pd.to_numeric, errors='coerce').dropna()
data2 = data2.apply(pd.to_numeric, errors='coerce').dropna()

# ウィルコクソンの順位和検定を実行
U_duration, p_duration = stats.mannwhitneyu(data1['duration'], data2['duration'], alternative='two-sided')
U_distance, p_distance = stats.mannwhitneyu(data1['distance'], data2['distance'], alternative='two-sided')

# 結果の表示
print("移動時間のウィルコクソン検定結果:")
print("U値: ", U_duration)
print("p値: ", p_duration)

print("移動距離のウィルコクソン検定結果:")
print("U値: ", U_distance)
print("p値: ", p_distance)
