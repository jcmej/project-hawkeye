#import <Foundation/Foundation.h>
#import <CoreGraphics/CoreGraphics.h>
#import <CoreVideo/CoreVideo.h>

// Pure Objective-C interface (no OpenCV types) so Swift can import it
// through the bridging header.

NS_ASSUME_NONNULL_BEGIN

/// One detected ArUco marker. Corners are image pixel coordinates in
/// OpenCV order relative to the printed marker: c0 top-left, c1 top-right,
/// c2 bottom-right, c3 bottom-left. The marker's "top" edge = car's front.
@interface ArucoMarker : NSObject
@property (nonatomic, readonly) int markerId;
@property (nonatomic, readonly) CGPoint c0;
@property (nonatomic, readonly) CGPoint c1;
@property (nonatomic, readonly) CGPoint c2;
@property (nonatomic, readonly) CGPoint c3;
- (instancetype)initWithId:(int)markerId
                        c0:(CGPoint)c0
                        c1:(CGPoint)c1
                        c2:(CGPoint)c2
                        c3:(CGPoint)c3;
@end

/// Detects DICT_4X4_50 markers in the luma plane of a camera frame.
/// Not thread-safe: call from one queue only.
@interface ArucoDetectorBridge : NSObject
- (NSArray<ArucoMarker *> *)detectInPixelBuffer:(CVPixelBufferRef)pixelBuffer
    NS_SWIFT_NAME(detect(pixelBuffer:));
@end

/// Markerless obstacle detection by comparing each frame to a stored
/// empty-arena background, in a top-down (bird's-eye) view of the floor.
/// Not thread-safe: call from one queue only.
@interface ChangeDetectorBridge : NSObject
@property (nonatomic, readonly) BOOL hasBackground;

/// Stores the current frame as the empty-arena background.
/// `matrix` (9 numbers, row-major) maps image pixels to the top-down image,
/// which is cols*cellPixels wide and rows*cellPixels tall.
- (BOOL)captureBackgroundFromPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                  matrix:(NSArray<NSNumber *> *)matrix
                                    cols:(int)cols
                                    rows:(int)rows
                              cellPixels:(int)cellPixels
    NS_SWIFT_NAME(captureBackground(pixelBuffer:matrix:cols:rows:cellPixels:));

- (void)clearBackground;

/// Returns cols*rows bytes in world row order (row 0 = y 0).
/// Bit 0 = occupied (changed vs background), bit 1 = visible to this camera.
/// Returns nil if no background has been captured.
/// Pass `matrix` to warp this frame with an updated mapping (AR mode, where the
/// mapping tracks tiny phone movements); pass nil to reuse the capture-time one.
- (nullable NSData *)occupancyForPixelBuffer:(CVPixelBufferRef)pixelBuffer
                                   threshold:(int)threshold
                          minChangedFraction:(double)minChangedFraction
                                      matrix:(nullable NSArray<NSNumber *> *)matrix
    NS_SWIFT_NAME(occupancy(pixelBuffer:threshold:minChangedFraction:matrix:));
@end

NS_ASSUME_NONNULL_END
