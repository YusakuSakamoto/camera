#MEAN SHIFT CLUSTERING(平均値シフト法による領域分割)

##ステップ1(smoothing:平滑化)
まず探索点(j,i)におけるチャンネル1、チャンネル2、チャンネル3をそれぞれL,U,Vとして格納する。  

探索半径内にある対象点を(j2,i2)として、チャンネル1、チャンネル2、チャンネル3をそれぞれL2,U2,V2として格納する。  

チャンネル1、チャンネル2、チャンネル3のそれぞれの値の差を取り、あるしきい値(color_radius2よりも小さいときに)x,y,L,u,vを足し合わせる。足した座標の数をnumとしてインクリメントする。  

点の平均の座標を出し、モーメントを求める。  

shift = (座標上の距離の自乗) + (色距離の自乗)を計算して、ループが100回を超えるか、shiftの値が3よりも小さくなったところでループを終了する。  
  
ループを終了した時点でのL,U,Vを探索点に代入する。  
  
以上の処理を全点繰り返す。  
ステップ1は終了。  
  
効果:画像のスムージング  
  
###ステップ1補足  
1.fとは、float型の1の事を表している。  
  
##ステップ2(connect:接合)  
引数のlabelsに情報を代入するため、最初にまず代入する。  
  
色距離があるしきい値よりも小さい場合、かつラベルがついていない場合に、modeにL,u,vを格納する。  
  
8近傍のデータをループした後、該当した数で割って平均値を改めてmodeに格納する  
  
効果:labelsにlabelをつけながら、modeにL,u,vを代入する。  

##ステップ3(TransitiveClosure:推移閉包)
1. Build RAM using classifiction structure  
2. Treat each region Ri as a disjoint set  
3. Union Find  
4. Traverse joint sets, relabeling image.  

日本語訳  
1. 分類構造体を使用するRAMを立ち上げる。  
2. 各々の領域Riを共通の要素をもたないセットとみなしてください  
3. 要素発見  
4. 共同のセットを同一化し、再ラベリングを行う。  