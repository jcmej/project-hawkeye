// OpenCV headers must be imported BEFORE any Apple headers, otherwise
// Apple macros (e.g. NO, YES) collide with OpenCV identifiers.
#ifdef __cplusplus
#import <opencv2/core.hpp>
#import <opencv2/imgproc.hpp>
#import <opencv2/objdetect/aruco_detector.hpp>   // requires OpenCV 4.7+
#endif

#import "ArucoBridge.h"
#include <memory>
#include <vector>

// MARK: - Helpers

static bool isNV12(CVPixelBufferRef pb) {
    OSType fmt = CVPixelBufferGetPixelFormatType(pb);
    return fmt == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange ||
           fmt == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange;
}

/// Warps the grayscale (luma) plane into the top-down floor image.
static bool warpLuma(CVPixelBufferRef pb, const cv::Mat &M, const cv::Size &size,
                     cv::Mat &out, cv::Size *srcSize) {
    if (!isNV12(pb)) { return false; }
    bool ok = false;
    CVPixelBufferLockBaseAddress(pb, kCVPixelBufferLock_ReadOnly);
    void *base = CVPixelBufferGetBaseAddressOfPlane(pb, 0);
    if (base != nullptr) {
        try {
            cv::Mat gray((int)CVPixelBufferGetHeightOfPlane(pb, 0),
                         (int)CVPixelBufferGetWidthOfPlane(pb, 0),
                         CV_8UC1, base, CVPixelBufferGetBytesPerRowOfPlane(pb, 0));
            cv::warpPerspective(gray, out, M, size, cv::INTER_LINEAR,
                                cv::BORDER_CONSTANT, cv::Scalar(0));
            if (srcSize) { *srcSize = gray.size(); }
            ok = true;
        } catch (const cv::Exception &e) {
            NSLog(@"warp error: %s", e.what());
        }
    }
    CVPixelBufferUnlockBaseAddress(pb, kCVPixelBufferLock_ReadOnly);
    return ok;
}

// MARK: - ArucoMarker

@implementation ArucoMarker
- (instancetype)initWithId:(int)markerId
                        c0:(CGPoint)c0
                        c1:(CGPoint)c1
                        c2:(CGPoint)c2
                        c3:(CGPoint)c3 {
    if ((self = [super init])) {
        _markerId = markerId;
        _c0 = c0;
        _c1 = c1;
        _c2 = c2;
        _c3 = c3;
    }
    return self;
}
@end

// MARK: - ArucoDetectorBridge

@implementation ArucoDetectorBridge {
    std::unique_ptr<cv::aruco::ArucoDetector> _detector;
}

- (instancetype)init {
    if ((self = [super init])) {
        cv::aruco::Dictionary dict =
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::aruco::DetectorParameters params;
        // Sub-pixel corner refinement gives steadier heading estimates.
        params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        // Tuned for small markers seen at a low angle (tested on real footage):
        // more adaptive-threshold window sizes (default 3..23 step 10 = 3 passes),
        params.adaptiveThreshWinSizeMin = 3;
        params.adaptiveThreshWinSizeMax = 33;
        params.adaptiveThreshWinSizeStep = 6;
        // looser quadrilateral fitting for perspective-squashed markers,
        params.polygonalApproxAccuracyRate = 0.05;
        // and more samples per bit when reading the code.
        params.perspectiveRemovePixelPerCell = 8;
        params.perspectiveRemoveIgnoredMarginPerCell = 0.2;
        _detector = std::make_unique<cv::aruco::ArucoDetector>(dict, params);
    }
    return self;
}

- (NSArray<ArucoMarker *> *)detectInPixelBuffer:(CVPixelBufferRef)pixelBuffer {
    NSMutableArray<ArucoMarker *> *result = [NSMutableArray array];
    if (!isNV12(pixelBuffer)) { return result; }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    void *base = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
    if (base != nullptr) {
        try {
            // Plane 0 of NV12 is grayscale luma; wrap it without copying.
            cv::Mat gray((int)CVPixelBufferGetHeightOfPlane(pixelBuffer, 0),
                         (int)CVPixelBufferGetWidthOfPlane(pixelBuffer, 0),
                         CV_8UC1, base, CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0));
            _detector->detectMarkers(gray, corners, ids, rejected);
        } catch (const cv::Exception &e) {
            NSLog(@"ArUco detection error: %s", e.what());
        }
    }
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    for (size_t i = 0; i < ids.size() && i < corners.size(); i++) {
        const auto &c = corners[i];
        if (c.size() != 4) { continue; }
        [result addObject:[[ArucoMarker alloc]
            initWithId:ids[i]
                    c0:CGPointMake(c[0].x, c[0].y)
                    c1:CGPointMake(c[1].x, c[1].y)
                    c2:CGPointMake(c[2].x, c[2].y)
                    c3:CGPointMake(c[3].x, c[3].y)]];
    }
    return result;
}

@end

// MARK: - ChangeDetectorBridge

@implementation ChangeDetectorBridge {
    cv::Mat _M;            // 3x3 CV_64F, image pixels -> top-down image
    cv::Size _size;        // top-down image size
    cv::Mat _background;   // blurred top-down grayscale of the empty arena
    cv::Mat _valid;        // 255 where this camera actually sees the floor
    int _cols;
    int _rows;
    BOOL _has;
}

- (BOOL)hasBackground { return _has; }

- (void)clearBackground {
    _has = NO;
    _background.release();
    _valid.release();
}

- (BOOL)captureBackgroundFromPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                  matrix:(NSArray<NSNumber *> *)matrix
                                    cols:(int)cols
                                    rows:(int)rows
                              cellPixels:(int)cellPixels {
    if (matrix.count != 9 || cols <= 0 || rows <= 0 || cellPixels <= 0) { return NO; }

    cv::Mat M(3, 3, CV_64F);
    for (int i = 0; i < 9; i++) { M.at<double>(i / 3, i % 3) = matrix[i].doubleValue; }
    cv::Size size(cols * cellPixels, rows * cellPixels);

    cv::Mat top;
    cv::Size src;
    if (!warpLuma(pixelBuffer, M, size, top, &src)) { return NO; }

    try {
        cv::GaussianBlur(top, _background, cv::Size(5, 5), 0);
        // Which top-down pixels come from inside the camera image?
        cv::Mat ones(src, CV_8UC1, cv::Scalar(255));
        cv::warpPerspective(ones, _valid, M, size, cv::INTER_NEAREST,
                            cv::BORDER_CONSTANT, cv::Scalar(0));
        cv::erode(_valid, _valid, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    } catch (const cv::Exception &e) {
        NSLog(@"background capture error: %s", e.what());
        return NO;
    }

    _M = M;
    _size = size;
    _cols = cols;
    _rows = rows;
    _has = YES;
    return YES;
}

- (nullable NSData *)occupancyForPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                   threshold:(int)threshold
                          minChangedFraction:(double)minChangedFraction
                                      matrix:(nullable NSArray<NSNumber *> *)matrix {
    if (!_has) { return nil; }

    cv::Mat M = _M;
    if (matrix != nil && matrix.count == 9) {
        M = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; i++) { M.at<double>(i / 3, i % 3) = matrix[i].doubleValue; }
    }

    cv::Mat top;
    if (!warpLuma(pixelBuffer, M, _size, top, nullptr)) { return nil; }

    cv::Mat frac, vis;
    try {
        cv::GaussianBlur(top, top, cv::Size(5, 5), 0);

        // Compensate for small global brightness drift (exposure is locked,
        // but room lighting can still shift a little).
        double shift = cv::mean(top, _valid)[0] - cv::mean(_background, _valid)[0];
        top.convertTo(top, -1, 1.0, -shift);

        cv::Mat diff, mask;
        cv::absdiff(top, _background, diff);
        cv::threshold(diff, mask, threshold, 255, cv::THRESH_BINARY);
        cv::bitwise_and(mask, _valid, mask);
        // Remove speckle noise and thin edge flicker.
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        // Area-averaging down to one pixel per cell = fraction of changed pixels.
        cv::resize(mask, frac, cv::Size(_cols, _rows), 0, 0, cv::INTER_AREA);
        cv::resize(_valid, vis, cv::Size(_cols, _rows), 0, 0, cv::INTER_AREA);
    } catch (const cv::Exception &e) {
        NSLog(@"occupancy error: %s", e.what());
        return nil;
    }

    NSMutableData *out = [NSMutableData dataWithLength:(NSUInteger)(_cols * _rows)];
    uint8_t *bytes = (uint8_t *)out.mutableBytes;
    const double occT = minChangedFraction * 255.0;
    const double visT = 0.9 * 255.0;
    for (int r = 0; r < _rows; r++) {
        int worldRow = _rows - 1 - r;   // top-down image row 0 is the +y edge
        for (int c = 0; c < _cols; c++) {
            bool visible = vis.at<uint8_t>(r, c) >= visT;
            uint8_t v = visible ? 2 : 0;
            if (visible && frac.at<uint8_t>(r, c) >= occT) { v |= 1; }
            bytes[worldRow * _cols + c] = v;
        }
    }
    return out;
}

@end
