import time
from vuer import Vuer
from vuer.events import ClientEvent
from vuer.schemas import ImageBackground, group, Hands, WebRTCStereoVideoPlane, DefaultScene
from multiprocessing import Array, Process, shared_memory, Queue, Manager, Event, Semaphore
import numpy as np
import asyncio
from webrtc.zed_server import *

class OpenTeleVision:
    def __init__(self, img_shape, shm_name, queue, toggle_streaming, stream_mode="image", cert_file="./cert.pem", key_file="./key.pem", ngrok=False):
        # self.app=Vuer()
        self.img_shape = (img_shape[0], 2*img_shape[1], 3)
        self.img_height, self.img_width = img_shape[:2]

        if ngrok:
            self.app = Vuer(host='0.0.0.0', queries=dict(grid=False), queue_len=3)
        else:
            self.app = Vuer(host='0.0.0.0', cert=cert_file, key=key_file, queries=dict(grid=False), queue_len=3)

        self.app.add_handler("HAND_MOVE")(self.on_hand_move)
        self.app.add_handler("CAMERA_MOVE")(self.on_cam_move)
        if stream_mode == "image":
            existing_shm = shared_memory.SharedMemory(name=shm_name)
            self.img_array = np.ndarray((self.img_shape[0], self.img_shape[1], 3), dtype=np.uint8, buffer=existing_shm.buf)
            self.app.spawn(start=False)(self.main_image)
        elif stream_mode == "webrtc":
            self.app.spawn(start=False)(self.main_webrtc)
        else:
            raise ValueError("stream_mode must be either 'webrtc' or 'image'")

        self.left_hand_shared = Array('d', 16, lock=True)
        self.right_hand_shared = Array('d', 16, lock=True)
        self.left_landmarks_shared = Array('d', 75, lock=True)
        self.right_landmarks_shared = Array('d', 75, lock=True)
        
        self.head_matrix_shared = Array('d', 16, lock=True)
        self.aspect_shared = Value('d', 1.0, lock=True)
        if stream_mode == "webrtc":
            # webrtc server
            if Args.verbose:
                logging.basicConfig(level=logging.DEBUG)
            else:
                logging.basicConfig(level=logging.INFO)
            Args.img_shape = img_shape
            # Args.shm_name = shm_name
            Args.fps = 60

            ssl_context = ssl.SSLContext()
            ssl_context.load_cert_chain(cert_file, key_file)

            app = web.Application()
            cors = aiohttp_cors.setup(app, defaults={
                "*": aiohttp_cors.ResourceOptions(
                    allow_credentials=True,
                    expose_headers="*",
                    allow_headers="*",
                    allow_methods="*",
                )
            })
            rtc = RTC(img_shape, queue, toggle_streaming, 60)
            app.on_shutdown.append(on_shutdown)
            cors.add(app.router.add_get("/", index))
            cors.add(app.router.add_get("/client.js", javascript))
            cors.add(app.router.add_post("/offer", rtc.offer))

            self.webrtc_process = Process(target=web.run_app, args=(app,), kwargs={"host": "0.0.0.0", "port": 8080, "ssl_context": ssl_context})
            self.webrtc_process.daemon = True
            self.webrtc_process.start()
            # web.run_app(app, host="0.0.0.0", port=8080, ssl_context=ssl_context)

        self.process = Process(target=self.run)
        self.process.daemon = True
        self.process.start()

    
    def run(self):
        self.app.run()

    async def on_cam_move(self, event, session, fps=60):
        # only intercept the ego camera.
        # if event.key != "ego":
        #     return
        try:
            # with self.head_matrix_shared.get_lock():  # Use the lock to ensure thread-safe updates
            #     self.head_matrix_shared[:] = event.value["camera"]["matrix"]
            # with self.aspect_shared.get_lock():
            #     self.aspect_shared.value = event.value['camera']['aspect']
            self.head_matrix_shared[:] = event.value["camera"]["matrix"]
            self.aspect_shared.value = event.value['camera']['aspect']
        except:
            pass
        # self.head_matrix = np.array(event.value["camera"]["matrix"]).reshape(4, 4, order="F")
        # print(np.array(event.value["camera"]["matrix"]).reshape(4, 4, order="F"))
        # print("camera moved", event.value["matrix"].shape, event.value["matrix"])

    async def on_hand_move(self, event, session, fps=60):
        try:
            # left_landmarks = np.array(event.value["leftLandmarks"]).reshape(25, 3)
            # right_landmarks = np.array(event.value["rightLandmarks"]).reshape(25, 3)

            # self.left_hand_shared[:] = event.value["leftHand"]
            # self.right_hand_shared[:] = event.value["rightHand"]
            # self.left_landmarks_shared[:] = left_landmarks.flatten()
            # self.right_landmarks_shared[:] = right_landmarks.flatten()

            # # thumb to index distance
            # left_distance = np.linalg.norm(left_landmarks[4] - left_landmarks[8])
            # right_distance = np.linalg.norm(right_landmarks[4] - right_landmarks[8])
            # print(f"Left hand thumb–index distance: {left_distance:.3f} m")
            # print(f"Right hand thumb–index distance: {right_distance:.3f} m")
            self.left_hand_shared[:] = event.value["leftHand"]
            self.right_hand_shared[:] = event.value["rightHand"]
            self.left_landmarks_shared[:] = np.array(event.value["leftLandmarks"]).flatten()
            self.right_landmarks_shared[:] = np.array(event.value["rightLandmarks"]).flatten()
            # Wrist pos and orientation
            # left_hand_matrix = np.array(event.value["leftHand"]).reshape(4, 4, order="F")
            # right_hand_matrix = np.array(event.value["rightHand"]).reshape(4, 4, order="F")

            # left_pos = left_hand_matrix[:3, 3]
            # right_pos = right_hand_matrix[:3, 3]

            # from scipy.spatial.transform import Rotation as R
            # left_quat = R.from_matrix(left_hand_matrix[:3, :3]).as_quat()
            # right_quat = R.from_matrix(right_hand_matrix[:3, :3]).as_quat()

            # print(f"Left Wrist Position: {left_pos}")
            # print(f"Left Wrist Quaternion: {left_quat}")
            # print(f"Right Wrist Position: {right_pos}")
            # print(f"Right Wrist Quaternion: {right_quat}")
        except: 
            pass
    
    async def main_webrtc(self, session, fps=60):
        session.set @ DefaultScene(frameloop="always")
        session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=True, showRight=True)
        session.upsert @ WebRTCStereoVideoPlane(
                src="https://192.168.8.102:8080/offer",
                key="zed",
                aspect=1.33334,
                height = 8,
                position=[0, -2, -0.2],
            )
        while True:
            await asyncio.sleep(1)
    
    async def main_image(self, session, fps=60):
        session.upsert @ Hands(fps=fps, stream=True, key="hands", showLeft=True, showRight=True)
        while True:
            # aspect = self.aspect_shared.value
            display_image = self.img_array

            # session.upsert(
            # ImageBackground(
            #     # Can scale the images down.
            #     display_image[:self.img_height],
            #     # 'jpg' encoding is significantly faster than 'png'.
            #     format="jpeg",
            #     quality=80,
            #     key="left-image",
            #     interpolate=True,
            #     # fixed=True,
            #     aspect=1.778,
            #     distanceToCamera=2,
            #     position=[0, -0.5, -2],
            #     rotation=[0, 0, 0],
            # ),
            # to="bgChildren",
            # )

            session.upsert(
            [ImageBackground(
                # Can scale the images down.
                display_image[::2, :self.img_width],
                # display_image[:self.img_height:2, ::2],
                # 'jpg' encoding is significantly faster than 'png'.
                format="jpeg",
                quality=80,
                key="left-image",
                interpolate=True,
                # fixed=True,
                aspect=1.66667,
                # distanceToCamera=0.5,
                height = 8,
                position=[0, -1, 3],
                # rotation=[0, 0, 0],
                layers=1, 
                alphaSrc="./vinette.jpg"
            ),
            ImageBackground(
                # Can scale the images down.
                display_image[::2, self.img_width:],
                # display_image[self.img_height::2, ::2],
                # 'jpg' encoding is significantly faster than 'png'.
                format="jpeg",
                quality=80,
                key="right-image",
                interpolate=True,
                # fixed=True,
                aspect=1.66667,
                # distanceToCamera=0.5,
                height = 8,
                position=[0, -1, 3],
                # rotation=[0, 0, 0],
                layers=2, 
                alphaSrc="./vinette.jpg"
            )],
            to="bgChildren",
            )
            await asyncio.sleep(0.03)

    @property
    def left_hand(self):
        # with self.left_hand_shared.get_lock():
        #     return np.array(self.left_hand_shared[:]).reshape(4, 4, order="F")
        return np.array(self.left_hand_shared[:]).reshape(4, 4, order="F")
        
    
    @property
    def right_hand(self):
        # with self.right_hand_shared.get_lock():
        #     return np.array(self.right_hand_shared[:]).reshape(4, 4, order="F")
        return np.array(self.right_hand_shared[:]).reshape(4, 4, order="F")
        
    
    @property
    def left_landmarks(self):
        # with self.left_landmarks_shared.get_lock():
        #     return np.array(self.left_landmarks_shared[:]).reshape(25, 3)
        return np.array(self.left_landmarks_shared[:]).reshape(25, 3)
    
    @property
    def right_landmarks(self):
        # with self.right_landmarks_shared.get_lock():
            # return np.array(self.right_landmarks_shared[:]).reshape(25, 3)
        return np.array(self.right_landmarks_shared[:]).reshape(25, 3)

    @property
    def head_matrix(self):
        # with self.head_matrix_shared.get_lock():
        #     return np.array(self.head_matrix_shared[:]).reshape(4, 4, order="F")
        return np.array(self.head_matrix_shared[:]).reshape(4, 4, order="F")

    @property
    def aspect(self):
        # with self.aspect_shared.get_lock():
            # return float(self.aspect_shared.value)
        return float(self.aspect_shared.value)

    
if __name__ == "__main__":
    resolution = (720, 1280)
    crop_size_w = 340  # (resolution[1] - resolution[0]) // 2
    crop_size_h = 270
    resolution_cropped = (resolution[0] - crop_size_h, resolution[1] - 2 * crop_size_w)  # 450 * 600
    img_shape = (2 * resolution_cropped[0], resolution_cropped[1], 3)  # 900 * 600
    img_height, img_width = resolution_cropped[:2]  # 450 * 600
    shm = shared_memory.SharedMemory(create=True, size=np.prod(img_shape) * np.uint8().itemsize)
    shm_name = shm.name
    img_array = np.ndarray((img_shape[0], img_shape[1], 3), dtype=np.uint8, buffer=shm.buf)

    tv = OpenTeleVision(resolution_cropped, cert_file="../cert.pem", key_file="../key.pem")
    while True:
        # print(tv.left_landmarks)
        # print(tv.left_hand)
        # tv.modify_shared_image(random=True)
        time.sleep(1)
