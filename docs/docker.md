# Docker
## Ubuntu
- 

## Mac
### 準備
- Dockerのアカウントを作成し、Docker Hubにアクセスする。
  - https://hub.docker.com/
- Docker Desktop for Macをインストールする。
  - https://docs.docker.com/get-docker/
- Docker.appを起動し、IDとPasswordを入力する。
- ターミナルでDockerのバージョンを確認する。
  ```
  $ docker version
  ```
- Ubuntu 18.04のイメージをダウンロードする。
  ```
  $ docker pull ubuntu:18.04
  ```
- イメージが存在するかを確認する。
 $ docker images

### 起動
- コンテナーを立ち上げる。オプションnameでコンテナーに対して名前を付ける。
  ```
  $ docker run -it -d --name ubuntu ubuntu:18.04
  ```
- コンテナーが存在するかを確認する。
  ```
  $ docker ps
  ```
- コンテナーに入る。
  ```
  $ docker exec -it ubuntu /bin/bash
  ```
- ユーザーを作成する。
  ```
  $ adduser ユーザー名
  $ gpasswd -a user_name sudo
  $ su - ユーザー名
  ```
  - sudoコマンドをインストールする必要がある？
    ```
    $ apt install sudo
    ```

## Windows

