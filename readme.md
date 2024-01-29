speed.py：シミュレータ
path.py：攻撃車両が進路上に偽の通行不能箇所を配置するシミュレータ
result(csv)：実行結果の置き場
wilcoxon.py：result(csv)内の2つのcsvを比較
goal_time_hist：到着時間のグラフ
moving_distance_hist：移動距離のグラフ
gridn×n.net.xml：ネットワーク

speed.py及びpath.pyの24-27行目に車両数等のパラメータが用意されている。12行目にはseed値が設定されている。
これらを変更することでシミュレータの挙動を変えられる。
実行したいシミュレータのファイルを直接実行すれば起動する。
なお、path.pyは起動時に無限ループになる場合がある(特にnum_fakeobstaclesを大きくした場合)。その場合には終了・再起動が必要となる。