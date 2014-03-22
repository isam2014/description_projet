#include <cv.h>
#include <highgui.h>
#include <time.h>
#include <math.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

const double MHI_DURATION = 1;
const double MAX_TIME_DELTA = 1;
const double MIN_TIME_DELTA = 1;
const int N = 3;


const int CONTOUR_MAX_AERA = 5000;

// ring image buffer
IplImage **buf = 0;
int last = 0;

// temporary images
IplImage *mhi = 0; // MHI: motion history image

//CvFilter filter = CV_GAUSSIAN_5x5;
CvConnectedComp *cur_comp, min_comp;
CvConnectedComp comp;
CvMemStorage *storage;
CvPoint pt[4];


void  update_mhi( IplImage* img, IplImage* dst, int diff_threshold )
{
    double timestamp = clock()/100.; // get current time in seconds
    CvSize size = cvSize(img->width,img->height); // get current frame size
    int i, j, idx1, idx2, count;
    IplImage* silh;
	IplImage* imageTmp;
	IplImage* imageRGB;
    uchar val;
    float temp;
    IplImage* pyr = cvCreateImage( cvSize((size.width & -2)/2, (size.height & -2)/2), 8, 1 );
    CvMemStorage *stor;
    CvSeq *cont, *result, *squares;
    CvSeqReader reader;

    if( !mhi || mhi->width != size.width || mhi->height != size.height )
    {
        if( buf == 0 )
        {
            buf = (IplImage**)malloc(N*sizeof(buf[0]));
            memset( buf, 0, N*sizeof(buf[0]));
        }

        for( i = 0; i < N; i++ )
        {
            cvReleaseImage( &buf[i] );
            buf[i] = cvCreateImage( size, IPL_DEPTH_8U, 1 );
            cvZero( buf[i] );
        }
        cvReleaseImage( &mhi );
        mhi = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        cvZero( mhi ); // clear MHI at the beginning
    } // end of if(mhi)

    cvCvtColor( img, buf[last], CV_BGR2GRAY ); // convert frame to grayscale

    idx1 = last;
    idx2 = (last + 1) % N; // index of (last - (N-1))th frame
    last = idx2;

    // ×öÖ¡²î
    silh = buf[idx2];
    cvAbsDiff( buf[idx1], buf[idx2], silh ); // get difference between frames

    cvThreshold( silh, silh, 100, 255, CV_THRESH_BINARY ); // and threshold it

    cvUpdateMotionHistory( silh, mhi, timestamp, MHI_DURATION ); // update MHI
    cvCvtScale( mhi, dst, 255./MHI_DURATION,
      (MHI_DURATION - timestamp)*255./MHI_DURATION );
    cvCvtScale( mhi, dst, 255./MHI_DURATION, 0 );


    cvSmooth( dst, dst, CV_MEDIAN, 3, 0, 0, 0 );


    cvPyrDown( dst, pyr, 7 );
    cvDilate( pyr, pyr, 0, 1 );
    cvPyrUp( pyr, dst, 7 );


    stor = cvCreateMemStorage(0);
    cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
    cvShowImage( "Car", dst );

    cvFindContours( dst, stor, &cont, sizeof(CvContour),
                    CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

	count = 0;
    for(;cont;cont = cont->h_next)
    {
              CvRect r = ((CvContour*)cont)->rect;
			  if((r.height - r.width) > 20 && (r.height - r.width) < 50)
			  {
				  if(r.height * r.width > CONTOUR_MAX_AERA)
				  {
					  CvFont font;
					  cvInitFont(&font,CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0,1,CV_AA);
					  CvScalar textColor = CV_RGB(0,255,255);	// light blue text
				      char text[256];
					  cvRectangle( img, cvPoint(r.x,r.y),
							  cvPoint(r.x + r.width, r.y + r.height),
							  CV_RGB(255,0,0), 0, CV_AA,0);

						printf(text, sizeof(text)-1, "%d ; %d", r.x + r.width/2, r.y + r.height/2);
						cvPutText(img, text, cvPoint(r.x, r.y), &font, textColor);
						cvCircle( img, cvPoint(r.x + r.width/2 ,r.y + r.height/2), 3, CV_RGB(0,255,0), -1, 8, 0 );
						count++;

			  			IplImage *imageTmp;
						CvSize size;
						size.height = img->height;
						size.width = img->width;

						imageTmp = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
						cvCopy(img, imageTmp, NULL);


						cvSetImageROI(imageTmp, r);
						size.width = r.width;
						size.height = r.height;
						imageRGB = cvCreateImage(size, IPL_DEPTH_8U, img->nChannels);
						cvCopy(imageTmp, imageRGB, NULL);	// Copy just the region.
						//cvShowImage( "Car", imageRGB );
						cvReleaseImage( &imageTmp );

				  }

			  }
    }
    // free memory
    cvReleaseMemStorage(&stor);
    cvReleaseImage( &pyr );
}

int main(int argc, char** argv)
{
    IplImage* motion = 0;
    CvCapture* capture = 0;

    /*if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        capture = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
    else if( argc == 2 )
        capture = cvCaptureFromAVI( argv[1] ); */
	capture = cvCaptureFromAVI("car2.mp4");
    if( capture )
    {
        cvNamedWindow( "Motion", 1 );
		cvNamedWindow( "Car", 0 );
        for(;;)
        {
            IplImage* image;
            if( !cvGrabFrame( capture ))
                break;
            image = cvRetrieveFrame( capture ,0);
            if( image )
            {
                if( !motion )
                {
                    motion = cvCreateImage( cvSize(image->width,image->height), 8, 1 );
                    cvZero( motion );
                    motion->origin = image->origin;
                }
            }

            update_mhi( image, motion, 100 );
            cvShowImage( "Motion", image );

            if( cvWaitKey(10) >= 0 )
                break;
        }
        cvReleaseCapture( &capture );
        cvDestroyWindow( "Motion" );
    }
    return 0;
}
