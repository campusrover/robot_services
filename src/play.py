import redis

r = redis.Redis(host='redis-10221.c10.us-east-1-3.ec2.cloud.redislabs.com', port=10221, password='ROSlab134')
while True:
    cmdx = r.lpop("pito/Cmd")
    print(cmdx)


