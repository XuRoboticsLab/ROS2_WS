import roslibpy
import time
import statistics

def estimate_time_offset(client, samples=50):
    offsets = []
    rtts = []
    
    for i in range(samples):
        t_before = time.time()
        ros_time = client.get_time()
        t_after = time.time()
        
        ros_time_sec = ros_time.to_sec()
        print(t_before, ros_time_sec, t_after)
        rtt = t_after - t_before
        t_local_mid = t_before + rtt / 2
        offset = ros_time_sec - t_local_mid
        
        offsets.append(offset)
        rtts.append(rtt)
    
    median_offset = sorted(offsets)[len(offsets) // 2]
    
    print(f'=== 时钟偏差 ===')
    print(f'  均值:    {statistics.mean(offsets)*1000:.3f} ms')
    print(f'  中位数:  {median_offset*1000:.3f} ms')
    print(f'  标准差:  {statistics.stdev(offsets)*1000:.3f} ms')
    print(f'  最小值:  {min(offsets)*1000:.3f} ms')
    print(f'  最大值:  {max(offsets)*1000:.3f} ms')
    print()
    print(f'=== RTT (网络往返延迟) ===')
    print(f'  均值:    {statistics.mean(rtts)*1000:.3f} ms')
    print(f'  标准差:  {statistics.stdev(rtts)*1000:.3f} ms')
    print(f'  最小值:  {min(rtts)*1000:.3f} ms')
    print(f'  最大值:  {max(rtts)*1000:.3f} ms')
    
    return median_offset


client = roslibpy.Ros(host='localhost', port=9090)
client.run()
print('Connected:', client.is_connected)

offset = estimate_time_offset(client, samples=50)
client.terminate()