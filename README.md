# agri robo camera program
このプログラムは、１２月に開催されるトマトロボット協議会のために作成中のプログラムです。
opencvライブラリを使用して、画像処理のプログラムを制作中です。

##必要な環境
このプログラムは、raspberry piもしくは、ubuntuでの動作を前提としています。
raspberry pi において、普通に環境を整えるだけではc++11がサポートされていませんので、バージョン4.9以上にするために、次のコマンドを入力してください。  

`sudo apt-get install g++-4.9`
`sudo apt-get install c++11`

##raspberry PI にopencvを入れた経験語り
opencvの環境を必要としています。
最初の環境　Opencv-2.4.11  
stereo cameraを作成するときに、コンパイルが通らない、となって、opencvを入れなおしました  
`Opencv-2.4.11 ==>Opencv-3.0.0`
が、その場合、分割コンパイルによって実行ファイルを作成する段階で、「シンボルがありません」というエラーが出ます。
ネットサーフィンによってコンパイルオプションの順序がおかしいという記事を見つけましたが、どの順序を試してみてもうまく行かないので、ダウングレードしました。
`Opencv-3.0.0 ==> Opencv-2.4.11`  

この状態では、カメラキャリブレーション以外のプログラムは動きます。

ここで、何がエラーを引き起こしていたのかがわかりました。
ヘッダー群の下2つが悪さをしているようだ......
原因がわかったところで、もう一度opencv-3.0.0をインストール!

##現状
###arc_terminalについて
arc用のターミナル表示を実現するために、cloneした後に次のコマンドを入力してください
`make`  
`sudo make install`  

削除するときは、次のコマンドを入力してください
`sudo make remove`  

これでarc用のターミナル表示ができるようになりました。  
具体的な使用方法については、arc_sampleを参照してください。

###stereo_viewについて
とりあえず２つのカメラを用いて動画像の表示、保存ができるプログラムです。ほぼ作り捨てとなるでしょう。

###file_io_practiceについて
XML/YMLのファイルに書き出し、読み出しができるようなプログラムです。

###trackerについて
kalman_filterを用いた、トマトを追跡するプログラムです。なかなかのトマト判定能力を持っているようです。

###calibrationについて
カメラのキャリブレーション、平行化を行うプログラムです。
今からマッチングのアルゴリズムを開発していきます。

よろしくお願いします。
