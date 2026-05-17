import roslibpy
import numpy as np
import matplotlib.pyplot as plt
import base64
import argparse


def decode_image(msg):
    encoding = msg.get("encoding", "rgb8")
    height = msg["height"]
    width = msg["width"]
    step = msg["step"]

    raw = msg.get("data", "")
    if isinstance(raw, str):
        data = np.frombuffer(base64.b64decode(raw), dtype=np.uint8)
    else:
        data = np.array(raw, dtype=np.uint8)

    img = data.reshape((height, step))[:, : width * (step // width)]

    if encoding in ("rgb8",):
        img = img.reshape((height, width, 3))
    elif encoding in ("bgr8",):
        img = img.reshape((height, width, 3))[:, :, ::-1]
    elif encoding in ("rgba8",):
        img = img.reshape((height, width, 4))
    elif encoding in ("mono8", "8UC1"):
        img = img.reshape((height, width))
    elif encoding in ("16UC1", "mono16"):
        data16 = data.view(np.uint16).reshape((height, width))
        img = (data16 / data16.max() * 255).astype(np.uint8)
    else:
        img = img.reshape((height, width, -1))

    return img, encoding


def main(host="localhost", port=9090, topic="/extern/color/image_raw"):
    client = roslibpy.Ros(host=host, port=port)
    client.run()
    print(f"Connected: {client.is_connected}")

    plt.ion()
    fig, ax = plt.subplots()
    im_handle = [None]

    def callback(msg):
        try:
            img, encoding = decode_image(msg)
            if im_handle[0] is None:
                cmap = "gray" if img.ndim == 2 else None
                im_handle[0] = ax.imshow(img, cmap=cmap)
                ax.axis("off")
            else:
                im_handle[0].set_data(img)
            ax.set_title(f"{topic}  [{encoding}]")
            fig.canvas.flush_events()
            plt.pause(0.001)
        except Exception as e:
            print(f"Error decoding image: {e}")

    listener = roslibpy.Topic(client, topic, "sensor_msgs/Image")
    listener.subscribe(callback)
    print(f"Subscribed to {topic}. Press Ctrl+C to quit.")

    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        pass
    finally:
        listener.unsubscribe()
        client.terminate()
        print("Disconnected.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="View a ROS Image topic via rosbridge.")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--topic", default="/extern/color/image_raw")
    args = parser.parse_args()
    main(host=args.host, port=args.port, topic=args.topic)
