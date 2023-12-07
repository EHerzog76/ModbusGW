#https://forums.docker.com/t/how-to-add-requirement-txt/134767/2
#https://www.docker.com/blog/containerized-python-development-part-1/
#https://www.docker.com/blog/containerized-python-development-part-2/
#https://www.docker.com/blog/containerized-python-development-part-3/
#
#############################
# First Stage
#FROM python:3.10.13-bullseye AS builder
FROM python:3.10.13-alpine AS builder

### Set up and activate virtual environment
ENV VIRTUAL_ENV "/venv"
RUN python3 -m venv $VIRTUAL_ENV && ${VIRTUAL_ENV}/bin/pip install -U pip
#RUN python3 -m venv --copies $VIRTUAL_ENV && cd ${VIRTUAL_ENV}/bin/ && chmod a+x activate && ./activate && chmod a-x activate && cd -
ENV PATH "$VIRTUAL_ENV/bin:$PATH"

COPY requirements.txt .

# install dependencies
#   use:  --user   to install in  /root/.local
#	--no-cache-dir --no-compile
RUN ${VIRTUAL_ENV}/bin/pip uninstall -y serial pyserial && \
	${VIRTUAL_ENV}/bin/pip install -r requirements.txt
#	pip3 install --no-cache-dir --no-compile pipenv && \
#	PIPENV_VENV_IN_PROJECT=1 pip install -r requirements.txt


#############################
# Second Stage
LABEL org.opencontainers.image.authors="e.herzog76@live.de"
#FROM python:3.10.13-slim-bullseye
FROM python:3.10.13-alpine

ENV VIRTUAL_ENV "/venv"
RUN mkdir /app
#WORKDIR /app


### Copy only the dependencies installation from the 1st stage image
#COPY --from=builder /root/.local /root/.local
COPY --from=builder $VIRTUAL_ENV $VIRTUAL_ENV
COPY ./src /app


# update PATH environment variable
#ENV PATH=/root/.local/bin:$PATH
ENV PATH "$VIRTUAL_ENV/bin:$PATH"
ENV PYTHONPATH="${VIRTUAL_ENV}/lib/python3.10/site-packages/"

#ENV PYTHONDONTWRITEBYTECODE 1
#ENV PYTHONUNBUFFERED 1

ENV SERIAL_DEVICE=/dev/ttyUSB0
ENV LISTEN_ADDRESS=0.0.0.0
ENV PORT=502
ENV SERIAL_BAUDRATE=19200
ENV SERIAL_BYTESIZE=8
ENV SERIAL_PARITY=N
ENV SERIAL_STOPBITS=1
ENV SERIAL_FRAMETYPE=RTU
ENV SERIAL_TIMEOUT=1.0
ENV SERIAL_WAITTIME=0
ENV SERIAL_EOF=0.05
ENV MQTT_HOST=
ENV MQTT_PORT=1883
ENV MQTT_TOPIC=modbus
ENV MQTT_CLIENTID=pyRS485GW
ENV MQTT_USER=usr
ENV MQTT_PWD=***
ENV MQTT_VERSION=MQTTv5
ENV LOG_LEVEL=INFO
#ENV DEBUG=False

#EXPOSE 502/tcp

###Test-Output:
#RUN ls -alh /venv/bin/ && \
#	ls -alh /venv/lib/python3.10/site-packages/ && \
#	ls -alh /app && \
#	ls -alh /venv/lib/python3.10/site-packages/serial/ && \
#	cat /venv/lib/python3.10/site-packages/serial/__init__.py

ENTRYPOINT ["/venv/bin/python3", "-m", "/app/pyModbusGW"]
CMD [ "/dev/ttyUSB0"]
