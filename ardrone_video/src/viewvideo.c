#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <pthread.h>
#include <stdint.h>

#define DOWNSIZE_FACTOR   8

#define VIDEO_DEVICE "/dev/video1"
#define VIDEO_DEVICE_WIDTH 1280
#define VIDEO_DEVICE_HEIGHT 720
#define CLEAR(x) memset(&(x), 0, sizeof (x))
/*
struct video_config_t front_camera = {
		.output_size = {
				.w = 1280,
				.h = 720
		},
		.sensor_size = {
				.w = 1280,
				.h = 720
		},
		.crop = {
				.x = 0,
				.y = 0,
				.w = 1280,
				.h = 720
		},
		.dev_name = "/dev/video1",
		.subdev_name = NULL,
		.format = V4L2_PIX_FMT_UYVY,
		.buf_cnt = 10,
		.filters = 0,
		.cv_listener=NULL,
		.fps = 0
};
*/

/* Image size structure */
struct img_size_t {
	uint16_t w;     ///< The width
	uint16_t h;     ///< The height
};

/* Image crop structure */
struct crop_t {
	uint16_t x;    ///< Start position x (horizontal)
	uint16_t y;    ///< Start position y (vertical)
	uint16_t w;    ///< Width of the cropped area
	uint16_t h;    ///< height of the cropped area
};

#define V4L2_IMG_NONE 255  ///< There currently no image available
enum image_type {
	IMAGE_YUV422,     ///< UYVY format (uint16 per pixel)
	IMAGE_GRAYSCALE,  ///< Grayscale image with only the Y part (uint8 per pixel)
	IMAGE_JPEG,       ///< An JPEG encoded image (not per pixel encoded)
	IMAGE_GRADIENT    ///< An image gradient (int16 per pixel)
};
struct FloatEulers {
  float phi; ///< in radians
  float theta; ///< in radians
  float psi; ///< in radians
};
struct image_t {
	enum image_type type;   ///< The image type
	uint16_t w;             ///< Image width
	uint16_t h;             ///< Image height
	struct timeval ts;      ///< The timestamp of creation
	struct FloatEulers eulers;   ///< Euler Angles at time of image
	uint32_t pprz_ts;       ///< The timestamp in us since system startup

	uint8_t buf_idx;        ///< Buffer index for V4L2 freeing
	uint32_t buf_size;      ///< The buffer size
	void *buf;              ///< Image buffer (depending on the image_type)
};

/* V4L2 memory mapped image buffer */
struct v4l2_img_buf {
	size_t length;              ///< The size of the buffer
	struct timeval timestamp;   ///< The time value of the image
	uint32_t pprz_timestamp;    ///< The time of the image in us since system startup
	void *buf;                  ///< Pointer to the memory mapped buffer
};

/* V4L2 device */
struct v4l2_device {
	char *name;                       ///< The name of the device
	int fd;                           ///< The file pointer to the device
	pthread_t thread;                 ///< The thread that handles the images
	uint16_t w;                       ///< The width of the image
	uint16_t h;                       ///< The height of the image
	uint8_t buffers_cnt;              ///< The number of image buffers
	volatile uint8_t buffers_deq_idx; ///< The current dequeued index
	pthread_mutex_t mutex;            ///< Mutex lock for enqueue/dequeue of buffers (change the deq_idx)
	struct v4l2_img_buf *buffers;     ///< The memory mapped image buffers
};


struct v4l2_device *v4l2_init(char *device_name, struct img_size_t size, struct crop_t crop, uint8_t buffers_cnt,
		uint32_t _pixelformat)
{
	uint8_t i;
	struct v4l2_capability cap;
	struct v4l2_format fmt;
	struct v4l2_requestbuffers req;
	struct v4l2_fmtdesc fmtdesc;
	struct v4l2_crop crp;
	CLEAR(cap);
	CLEAR(fmt);
	CLEAR(req);
	CLEAR(fmtdesc);
	CLEAR(crp);

	// Try to open the device
	int fd = open(device_name, O_RDWR | O_NONBLOCK, 0);
	if (fd < 0) {
		printf("[v4l2] Cannot open '%s': %d, %s\n", device_name, errno, strerror(errno));
		return NULL;
	}

	// Try to fetch the capabilities of the V4L2 device
	if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
		printf("[v4l2] %s is no V4L2 device\n", device_name);
		close(fd);
		return NULL;
	}

	// Check if the device is capable of capturing and streaming
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		printf("[v4l2] %s is no V4L2 video capturing device\n", device_name);
		close(fd);
		return NULL;
	}
	if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		printf("[v4l2] %s isn't capable of streaming (TODO: support reading)\n", device_name);
		close(fd);
		return NULL;
	}

	fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
		fmtdesc.index++;
		if(fmtdesc.pixelformat == _pixelformat)
			break;
	}

	// Accept if no format can be get
	if(fmtdesc.index != 0 && fmtdesc.pixelformat != _pixelformat) {
		printf("[v4l2] Pixelformat not available on device %s (wanted: %4X)\r\n", device_name, _pixelformat);
		return NULL;
	}

	// Set the cropping window
	crp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	crp.c.top = crop.y;
	crp.c.left = crop.x;
	crp.c.width = crop.w;
	crp.c.height = crop.h;

	// Only crop when needed
	if(crop.x != 0 || crop.y != 0 || crop.w != size.w || crop.h != size.h) {
		if (ioctl(fd, VIDIOC_S_CROP, &crp) < 0) {
			printf("[v4l2] Could not set crop window of %s\n", device_name);
			close(fd);
			return NULL;
		}
	}

	// Set the format settings
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = size.w;
	fmt.fmt.pix.height = size.h;
	fmt.fmt.pix.pixelformat = _pixelformat;
	fmt.fmt.pix.colorspace = V4L2_COLORSPACE_REC709;
	fmt.fmt.pix.field = V4L2_FIELD_NONE;

	if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
		printf("[v4l2] Could not set data format settings of %s\n", device_name);
		close(fd);
		return NULL;
	}

	// Request MMAP buffers
	req.count = buffers_cnt;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
		printf("[v4l2] %s Does not support memory mapping\n", device_name);
		close(fd);
		return NULL;
	}

	// Allocate memory for the memory mapped buffers
	struct v4l2_img_buf *buffers = calloc(req.count, sizeof(struct v4l2_img_buf));
	if (buffers == NULL) {
		printf("[v4l2] Not enough memory for %s to initialize %d MMAP buffers\n", device_name, req.count);
		close(fd);
		return NULL;
	}

	// Go trough the buffers and initialize them
	for (i = 0; i < req.count; ++i) {
		struct v4l2_buffer buf;
		CLEAR(buf);

		// Request the buffer information
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
			printf("[v4l2] Querying buffer %d from %s failed\n", i, device_name);
			free(buffers);
			close(fd);
			return NULL;
		}

		//  Map the buffer
		buffers[i].length = buf.length;
		buffers[i].buf = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
		if (MAP_FAILED == buffers[i].buf) {
			printf("[v4l2] Mapping buffer %d with length %d from %s failed\n", i, buf.length, device_name);
			free(buffers);
			close(fd);
			return NULL;
		}
	}

	// Create the device only when everything succeeded
	struct v4l2_device *dev = (struct v4l2_device *)malloc(sizeof(struct v4l2_device));
	CLEAR(*dev);
	dev->name = strdup(device_name); // NOTE: needs to be freed
	dev->fd = fd;
	dev->w = size.w;
	dev->h = size.h;
	dev->buffers_cnt = req.count;
	dev->buffers = buffers;
	return dev;
}

void v4l2_image_get(struct v4l2_device *dev, struct image_t *img)
{
	uint16_t img_idx = V4L2_IMG_NONE;

	// Continu to wait for an image
	while (img_idx == V4L2_IMG_NONE) {
		// We first check if the deq_idx is ok, this reduces the amount of locks
		if (dev->buffers_deq_idx != V4L2_IMG_NONE) {
			pthread_mutex_lock(&dev->mutex);

			// We need to check it here again, because it could be changed
			if (dev->buffers_deq_idx != V4L2_IMG_NONE) {
				img_idx = dev->buffers_deq_idx;
				dev->buffers_deq_idx = V4L2_IMG_NONE;
			}

			pthread_mutex_unlock(&dev->mutex);
		}
	}

	// Set the image
	img->type = IMAGE_YUV422;
	img->w = dev->w;
	img->h = dev->h;
	img->buf_idx = img_idx;
	img->buf_size = dev->buffers[img_idx].length;
	img->buf = dev->buffers[img_idx].buf;
	img->ts = dev->buffers[img_idx].timestamp;
	img->pprz_ts =  dev->buffers[img_idx].pprz_timestamp;
}

void image_create(struct image_t *img, uint16_t width, uint16_t height, enum image_type type)
{
  // Set the variables
  img->type = type;
  img->w = width;
  img->h = height;

  // Depending on the type the size differs
  if (type == IMAGE_YUV422) {
    img->buf_size = sizeof(uint8_t) * 2 * width * height;
  } else if (type == IMAGE_JPEG) {
    img->buf_size = sizeof(uint8_t) * 2 * width * height;  // At maximum quality this is enough
  } else if (type == IMAGE_GRADIENT) {
    img->buf_size = sizeof(int16_t) * width * height;
  } else {
    img->buf_size = sizeof(uint8_t) * width * height;
  }

  img->buf = malloc(img->buf_size);
}

int main(int argc,char ** argv)
{
	printf("Starting video test program!\n");

	struct img_size_t size;
	struct crop_t crop;
	size.h = 720;
	size.w = 1280;
	crop.x = 0;
	crop.y = 0;
	crop.h = 720;
	crop.w = 1280;
	FILE * file_fd = fopen("test.yuv", "w");
	struct v4l2_device * dev = v4l2_init(VIDEO_DEVICE, size, crop, 10, V4L2_PIX_FMT_UYVY);
	struct image_t img;
	image_create(&img, 1280, 720, IMAGE_YUV422);
	v4l2_image_get(dev, &img);
	fwrite(img.buf, sizeof(uint8_t), img.buf_size, file_fd);

	close(dev->fd);
	fclose(file_fd);
	free(dev);

	return 0;
}
