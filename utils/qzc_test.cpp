#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

#define COL 140
#define ROW 140
int kadane(int* arr, int* start, int* finish, int n)
{
    // initialize sum, maxSum and
    int sum = 0, maxSum = INT_MIN, i;

    // Just some initial value to check
    // for all negative values case
    *finish = -1;

    // local variable
    int local_start = 0;

    for (i = 0; i < n; ++i) {
        sum += arr[i];
        if (sum < 0) {
            sum         = 0;
            local_start = i + 1;
        }
        else if (sum > maxSum) {
            maxSum  = sum;
            *start  = local_start;
            *finish = i;
        }
    }

    // There is at-least one
    // non-negative number
    if (*finish != -1)
        return maxSum;

    // Special Case: When all numbers
    // in arr[] are negative
    maxSum = arr[0];
    *start = *finish = 0;

    // Find the maximum element in array
    for (i = 1; i < n; i++) {
        if (arr[i] > maxSum) {
            maxSum = arr[i];
            *start = *finish = i;
        }
    }
    return maxSum;
}

// The main function that finds
// maximum sum rectangle in M[][]
void findMaxSum(int M[][COL])
{
    // Variables to store the final output
    int maxSum = INT_MIN, finalLeft, finalRight, finalTop, finalBottom;

    int left, right, i;
    int temp[ROW], sum, start, finish;

    // Set the left column
    for (left = 0; left < COL; ++left) {
        // Initialize all elements of temp as 0
        memset(temp, 0, sizeof(temp));

        // Set the right column for the left
        // column set by outer loop
        for (right = left; right < COL; ++right) {
            // Calculate sum between current left
            // and right for every row 'i'
            for (i = 0; i < ROW; ++i)
                temp[i] += M[i][right];

            // Find the maximum sum subarray in temp[].
            // The kadane() function also sets values
            // of start and finish. So 'sum' is sum of
            // rectangle between (start, left) and
            // (finish, right) which is the maximum sum
            // with boundary columns strictly as left
            // and right.
            sum = kadane(temp, &start, &finish, ROW);

            // Compare sum with maximum sum so far.
            // If sum is more, then update maxSum and
            // other output values
            if (sum > maxSum) {
                maxSum      = sum;
                finalLeft   = left;
                finalRight  = right;
                finalTop    = start;
                finalBottom = finish;
            }
        }
    }

    // Print final values
    cout << "(Top, Left) (" << finalTop << ", " << finalLeft << ")" << endl;
    cout << "(Bottom, Right) (" << finalBottom << ", " << finalRight << ")" << endl;
    // we will find the mid point of the diagonal of the rectangle so that in the end we can make a circle with keeping
    // that point as center of the circle. getting the coordinates of centre of circle.
    int a, b;
    a = (finalBottom + finalTop) / 2;
    b = (finalLeft + finalRight) / 2;
    cout << "centerof circle is " << b << " " << a << endl;
    return;
}

int main()
{
    Mat image = imread("Enter the Address"
                       "of Input Image",
                       IMREAD_GRAYSCALE);

    // we first reshape it to a smaller matrix of say (140,140) shape

    int down_width = 140, down_height = 140;
    Mat final;
    resize(image, final, Size(down_width, down_height), INTER_LINEAR);

    // Show Image inside a window with
    // the name provided

    imshow("Grey scale image  ", final);
    // we grayscale the array which makes our array into a 2d shape

    cout << "Blurring the image to smoothen the kadane process,we basically divide the whole matrix by 40 to smoothen "
            "the process."
         << endl;
    int i, j, imagemax;
    for (i = 0; i < 140; i++) {
        for (j = 0; j < 140; j++) {
            final[i][j] = final[i][j] / 40;
        }
    }

    // we subtract all values of array with its max value and add 1 to it.(In Kadane’s we need negative too to get
    // properly maximum sum area, else we may end up detecting full array as the sum). finding max element in the 2d
    // matrix.

    for (i = 0; i < 140; i++) {
        for (j = 0; j < 140; j++) {
            imagemax = max(imagemax, final[i][j]);
        }
    }
    // we subtract all values of array with its max value and add 1 to it.(In Kadane’s we need negative too to get
    // properly maximum sum area, else we may end up detecting full array as the sum).

    for (i = 0; i < 140; i++) {
        for (j = 0; j < 140; j++) {
            final[i][j] = final[i][j] - imagemax + 1;
        }
    }
    // Now we will pass the matrix in kadane algo

    findMaxSum(final);

    waitKey(0);

    return 0;
}