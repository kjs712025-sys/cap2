FROM python:3.11-slim

WORKDIR /app

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    ROBOT_HOST=0.0.0.0 \
    ROBOT_PORT=8000

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . ./

RUN chmod +x /app/start.sh

VOLUME ["/app/logs", "/app/captures"]

EXPOSE 8000

CMD ["/app/start.sh"]
