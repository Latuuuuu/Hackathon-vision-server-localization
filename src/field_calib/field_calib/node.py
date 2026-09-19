"""field_calib_node: table-edge extrinsic calibration with live debug images.

live (timer)          : edge search + residuals under the current TF (no optimization)
                        -> ~/live/overlay, warns when the edges drift (camera bumped)
~/calibrate (Trigger) : median of N frames -> full calibration, every band published to
                        ~/calib/{overlay,strips,residuals} and saved as PNG; result written to yaml.
                        The node never publishes map -> camera_link itself (rs_launch.py does);
                        apply the printed cam_tf.* launch args after checking the result.
"""
import datetime
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from . import core, debug_viz

VIEWS = ('overlay', 'strips', 'residuals')


def tf_to_rt(tf):
    tr, q = tf.translation, tf.rotation
    return core.quat_to_rot([q.x, q.y, q.z, q.w]), np.array([tr.x, tr.y, tr.z])


class FieldCalibNode(Node):
    def __init__(self):
        super().__init__('field_calib_node')
        dp = self.declare_parameter
        dp('image_topic', '/camera/camera/color/image_raw')
        dp('camera_info_topic', '/camera/camera/color/camera_info')
        dp('depth_topic', '/camera/camera/depth/image_rect_raw')
        dp('depth_info_topic', '/camera/camera/depth/camera_info')
        dp('world_frame', 'map')
        dp('link_frame', 'camera_link')
        dp('optical_frame', 'camera_color_optical_frame')
        dp('depth_frame', 'camera_depth_optical_frame')
        dp('field_file', '')
        dp('field.disabled_segments', '')
        dp('edge.step', 0.01)
        dp('edge.blur', 1.0)
        dp('edge.grad_thresh', 6.0)
        dp('edge.contrast_thresh', 20.0)
        dp('lm.delta', 1.5)
        dp('calib.frames', 30)
        dp('calib.bands', [80.0, 40.0, 20.0, 12.0])
        dp('calib.stage_delay', 1.0)
        dp('calib.output_dir', '~/.ros/field_calib')
        dp('live.enable', True)
        dp('live.period', 1.0)
        dp('live.band', 12.0)
        dp('live.warn_rms_px', 1.5)
        dp('live.warn_inlier_ratio', 0.7)
        dp('depth.enable', True)
        dp('depth.frames', 10)
        dp('tag.id', 1)
        dp('tag.size', 0.08)
        dp('tag.height', 0.2)
        dp('debug.window', True)

        field_file = self.p('field_file') or os.path.join(
            get_package_share_directory('field_calib'), 'config', 'field.yaml')
        with open(field_file) as f:
            field_cfg = yaml.safe_load(f)
        disabled = [s.strip() for s in self.p('field.disabled_segments').split(',') if s.strip()]
        if disabled:
            field_cfg['disabled_segments'] = disabled
        self.field = core.Field(field_cfg)
        self.get_logger().info(f'field {field_file}: segments {self.field.names}')

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.K = self.D = self.K_depth = None
        self.latest = None
        self.frames = None      # list while collecting for a calibration
        self.depth_frames = None
        self.calib_views = None
        self.live_views = None
        self.show_calib = False  # windows show the calibration result instead of the live view
        self.stop_display = False
        self.busy = False

        sensors = MutuallyExclusiveCallbackGroup()
        self.create_subscription(CameraInfo, self.p('camera_info_topic'), self.on_info, 10, callback_group=sensors)
        self.create_subscription(Image, self.p('image_topic'), self.on_image, 10, callback_group=sensors)
        if self.p('depth.enable'):
            self.create_subscription(CameraInfo, self.p('depth_info_topic'), self.on_depth_info, 10,
                                     callback_group=sensors)
            self.create_subscription(Image, self.p('depth_topic'), self.on_depth, 10, callback_group=sensors)
        self.pubs = {f'{kind}/{v}': self.create_publisher(Image, f'~/{kind}/{v}', 1)
                     for kind, vs in (('live', ('overlay',)), ('calib', VIEWS)) for v in vs}
        self.create_service(Trigger, '~/calibrate', self.on_calibrate,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(self.p('live.period'), self.on_timer, callback_group=MutuallyExclusiveCallbackGroup())
        if self.p('debug.window'):
            threading.Thread(target=self.display_loop, daemon=True).start()
            self.get_logger().info('debug window: press c to calibrate, l to return to the live view')
        self.get_logger().info('ready: ros2 service call /field_calib_node/calibrate std_srvs/srv/Trigger')

    def p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------------ sensors
    def on_info(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        self.D = np.array(msg.d)

    def on_depth_info(self, msg):
        self.K_depth = np.array(msg.k).reshape(3, 3)

    def on_image(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        with self.lock:
            self.latest = (img, msg.header)
            if self.frames is not None and len(self.frames) < self.p('calib.frames'):
                self.frames.append(img)

    def on_depth(self, msg):
        with self.lock:
            if self.depth_frames is None or len(self.depth_frames) >= self.p('depth.frames'):
                return
        d = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        scale = 0.001 if d.dtype == np.uint16 else 1.0  # RealSense 16UC1 is mm
        with self.lock:
            if self.depth_frames is not None:
                self.depth_frames.append(d.astype(np.float32) * scale)

    def lookup(self, parent, child):
        return tf_to_rt(self.tf_buffer.lookup_transform(parent, child, Time()).transform)

    def current_pose(self):
        R_wc, t_wc = self.lookup(self.p('world_frame'), self.p('optical_frame'))
        p = core.pose_from_world_cam(R_wc, t_wc)
        return np.append(p, 0.0) if self.field.fit_dx else p

    def publish_views(self, kind, views, header):
        for name, im in views.items():
            msg = self.bridge.cv2_to_imgmsg(im, 'bgr8')
            msg.header = header
            self.pubs[f'{kind}/{name}'].publish(msg)

    def blurred(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
        return cv2.GaussianBlur(gray, (0, 0), self.p('edge.blur'))

    # ------------------------------------------------------------------ live view / monitor
    def on_timer(self):
        with self.lock:
            latest, busy = self.latest, self.busy
        if latest is None or self.K is None:
            return
        img, header = latest
        if self.calib_views is not None:
            self.publish_views('calib', self.calib_views, header)  # keep the last result visible
        if busy or not self.p('live.enable'):
            return
        try:
            p = self.current_pose()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'TF not available: {e}', throttle_duration_sec=5.0)
            return
        stage = core.evaluate(self.blurred(img), self.field, p, self.K, self.D, self.p('live.band'),
                              self.p('edge.step'), self.p('edge.grad_thresh'), self.p('edge.contrast_thresh'),
                              self.p('lm.delta'))
        views = {'overlay': debug_viz.render_overlay(img, self.field, self.K, self.D, stage,
                                                     self.p('lm.delta'), 'live')}
        with self.lock:
            self.live_views = views
        self.publish_views('live', views, header)
        d = stage['diag']
        considered = d['status'] != core.OUT_OF_IMAGE
        acc = d['status'] == core.ACCEPTED
        rms = float(np.sqrt(np.mean(d['resid'][acc] ** 2))) if acc.any() else float('nan')
        ratio = acc.sum() / max(1, considered.sum())
        if not (rms <= self.p('live.warn_rms_px')) or ratio < self.p('live.warn_inlier_ratio'):
            self.get_logger().warn(
                f'table edges do not match the current extrinsic: inlier RMS {rms:.2f} px, '
                f'accepted {ratio * 100:.0f}% (occlusion, moved table or bumped camera?)',
                throttle_duration_sec=10.0)

    # ------------------------------------------------------------------ calibration
    def collect(self, timeout):
        with self.lock:
            self.frames = []
            self.depth_frames = [] if self.p('depth.enable') else None
        t0 = time.time()
        n_img, n_depth = self.p('calib.frames'), self.p('depth.frames')
        while time.time() - t0 < timeout:
            with self.lock:
                done = len(self.frames) >= n_img and (self.depth_frames is None or len(self.depth_frames) >= n_depth)
            if done:
                break
            time.sleep(0.05)
        with self.lock:
            frames, depth = self.frames, self.depth_frames
            self.frames = self.depth_frames = None
        return frames, depth

    def on_calibrate(self, request, response):
        del request
        response.success, response.message = self.try_calibrate()
        return response

    def try_calibrate(self):
        """Run one calibration unless one is already running. Returns (success, message)."""
        with self.lock:
            if self.busy:
                return False, 'calibration already running'
            self.busy = True
            self.show_calib = True
        try:
            return True, self.run_calibration()
        except Exception as e:  # noqa: BLE001
            msg = f'calibration failed: {e}'
            self.get_logger().error(msg)
            return False, msg
        finally:
            with self.lock:
                self.busy = False

    # ------------------------------------------------------------------ OpenCV windows
    def display_loop(self):
        """All HighGUI calls stay in this thread. Only the overlay is shown; strips / residuals are
        saved as PNG during calibration. Keys: c = calibrate, l = back to the live view."""
        name = 'field_calib'
        sized = False
        while rclpy.ok() and not self.stop_display:
            with self.lock:
                views = self.calib_views if (self.show_calib and self.calib_views) else self.live_views
            if views:
                im = views['overlay']
                if not sized:
                    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
                    scale = min(1.0, 900.0 / im.shape[0], 1600.0 / im.shape[1])
                    cv2.resizeWindow(name, int(im.shape[1] * scale), int(im.shape[0] * scale))
                    sized = True
                cv2.imshow(name, im)
            key = cv2.waitKey(50) & 0xFF
            if key == ord('c'):
                threading.Thread(target=self.try_calibrate, daemon=True).start()
            elif key == ord('l'):
                with self.lock:
                    if not self.busy:
                        self.show_calib = False
        cv2.destroyAllWindows()

    def run_calibration(self):
        log = self.get_logger().info
        if self.K is None:
            raise RuntimeError('no camera_info yet')
        p0 = self.current_pose()
        R_lo, t_lo = self.lookup(self.p('link_frame'), self.p('optical_frame'))
        frames, depth = self.collect(timeout=10.0)
        if len(frames) < 3:
            raise RuntimeError(f'only {len(frames)} frames received')
        med = np.median(np.array(frames), axis=0).astype(np.uint8)
        gray_u8 = cv2.cvtColor(med, cv2.COLOR_BGR2GRAY)
        header = self.latest[1]
        out_dir = os.path.expanduser(self.p('calib.output_dir'))
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        debug_dir = os.path.join(out_dir, stamp)
        os.makedirs(debug_dir, exist_ok=True)
        delta = self.p('lm.delta')

        def on_stage(stage):
            views = debug_viz.render_all(med, self.field, self.K, self.D, stage, delta, 'calib')
            for name, im in views.items():
                cv2.imwrite(os.path.join(debug_dir, f'band_{stage["index"]}_{stage["band"]:.0f}px_{name}.png'), im)
            self.calib_views = views
            self.publish_views('calib', views, header)
            time.sleep(self.p('calib.stage_delay'))  # let viewers see every stage

        log(f'calibrating from {len(frames)} frames, debug images -> {debug_dir}')
        p, stage = core.calibrate(self.blurred(med), self.field, p0, self.K, self.D, list(self.p('calib.bands')),
                                  self.p('edge.step'), self.p('edge.grad_thresh'),
                                  self.p('edge.contrast_thresh'), delta, callback=on_stage, log=log)

        names = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        tf0 = core.cam_tf_from_pose(p0, R_lo, t_lo)
        tf1 = core.cam_tf_from_pose(p, R_lo, t_lo)
        dmm, ddeg = core.pose_delta(p0, p)
        rows = core.segment_stats(self.field, stage['diag'])
        lines = ['per segment (last band):'] + core.format_segment_table(rows)
        lines.append('cam_tf        current   calibrated')
        for i, n in enumerate(names):
            lines.append(f'  {n:6s} {tf0[i]:10.4f} {tf1[i]:11.4f}')
        lines.append(f'camera moved {dmm:.1f} mm, rotation change {ddeg:.3f} deg')
        if self.field.fit_dx:
            lines.append(f'lower table x offset {p[6] * 1000:+.1f} mm')

        depth_result = None
        if depth and self.K_depth is not None:
            try:
                R_cd, t_cd = self.lookup(self.p('optical_frame'), self.p('depth_frame'))
                dmed = np.median(np.array(depth), axis=0)
                depth_result = core.depth_plane_check(dmed, self.K_depth, R_cd, t_cd, self.field, p)
            except Exception as e:  # noqa: BLE001
                lines.append(f'depth check failed: {e}')
        if depth_result:
            lines.append(f'depth plane: {depth_result["n_points"]} pts, inlier {depth_result["inlier_ratio"] * 100:.0f}%, '
                         f'normal vs edge solution {depth_result["tilt_deg"]:.3f} deg, '
                         f'camera height depth {depth_result["height_depth"]:.4f} m vs edge {depth_result["height_edge"]:.4f} m')
        lines += core.tag_check(gray_u8, [('current', p0), ('calibrated', p)], self.K, self.D,
                                self.p('tag.id'), self.p('tag.size'), self.p('tag.height'))
        launch = ' '.join(f'cam_tf.{n}:={v:.5f}' for n, v in zip(names, tf1))
        lines.append('launch args: ' + launch)

        result = {
            'calibrated_at': stamp,
            'cam_tf': {n: round(float(v), 5) for n, v in zip(names, tf1)},
            'cam_tf_before': {n: round(float(v), 5) for n, v in zip(names, tf0)},
            'lower_dx': round(float(p[6]), 5) if self.field.fit_dx else None,
            'quality': {
                'segments': {r['name']: {'accepted': r['counts'][core.ACCEPTED], 'samples': r['n'],
                                         'rms_px': round(r['rms'], 3), 'mean_px': round(r['mean'], 3)}
                             for r in rows},
                'depth_plane': ({k: (round(v, 4) if isinstance(v, float) else v) for k, v in depth_result.items()}
                                if depth_result else None),
            },
            'debug_dir': debug_dir,
        }
        for path in (os.path.join(debug_dir, 'cam_tf.yaml'), os.path.join(out_dir, 'cam_tf.yaml')):
            with open(path, 'w') as f:
                yaml.safe_dump(result, f, sort_keys=False)
        lines.append(f'result: {os.path.join(out_dir, "cam_tf.yaml")}')
        text = '\n'.join(lines)
        log('\n' + text)
        return text


def main():
    rclpy.init()
    node = FieldCalibNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
