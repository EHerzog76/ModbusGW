#!/usr/bin/env bash

docker build -t eherzog76/modbusgw .

docker image tag eherzog76/modbusgw 10.0.0.121:5000/eherzog76/modbusgw
docker image push 10.0.0.121:5000/eherzog76/modbusgw
