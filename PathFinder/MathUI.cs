using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework;

namespace SauronsRevenge.Object_Handling.PathFinder
{
    internal static class MathUi
    {
        /// <summary>
        /// according to the given two points get a line function, which passes the given two points
        /// </summary>
        /// <param name="ponit1"></param>
        /// <param name="point2"></param>
        /// <returns></returns>
        private static Vector2 GetLineFunc(Vector2 ponit1, Vector2 point2)
        {

            // line func is in form of y = a or x = a 
            if (Math.Abs(ponit1.X - point2.X) <= 0)
            {
                return new Vector2(1, ponit1.X);
            }
            else if (Math.Abs(ponit1.Y - point2.Y) <= 0)
            {
                return new Vector2(0, -ponit1.Y);
            }
            // when the line is not vertical to the axes, then the line funtion: y = ax + b
            var a = (ponit1.Y - point2.Y) / (ponit1.X - point2.X);
            // according to : y1 = ax1 + b and y2 = ax2 + b
            // then we get a = ( y1 - y2 ) / ( x1 - x2 ) 
            var b = ponit1.Y - a * ponit1.X;
            return new Vector2(a, b);
        }

        private static IEnumerable<Point> GetNodesUnderPoint(float xPos, float yPos, 
            IReadOnlyDictionary<Point, Point> exception = null)
        {
            var result = new List<Point>();
            var xIsInt = Math.Abs(xPos % 1) <= 0;
            var yIsInt = Math.Abs(yPos % 1) <= 0;

            // if there are 4 points share this join point
            if (xIsInt && yIsInt)
            {
                result.Add(new Point((int)xPos - 1, (int)yPos - 1));
                result.Add(new Point((int)xPos, (int)yPos - 1));
                result.Add(new Point((int)xPos - 1, (int)yPos));
                result.Add(new Point((int)xPos, (int)yPos));
            }
            //two points share this point
            //the share point is on the left/right side of these two points
            else if (!yIsInt && xIsInt)
            {
                result.Add(new Point((int)xPos - 1, (int)yPos));
                result.Add(new Point((int)xPos, (int)yPos));
            }
            //the share point is on the up/down side of these two points
            else if (yIsInt)
            {
                result.Add(new Point((int)xPos, (int)yPos - 1));
                result.Add(new Point((int)xPos, (int)yPos));
            }
            //this point belongs to only one point
            else
            {
                result.Add(new Point((int)xPos, (int)yPos));
            }

            // check if some points are in exception list
            if (exception != null && exception.Count > 0)
            {
                for (var i = 0; i < result.Count; i++)
                {
                    if (exception.ContainsKey(result[i]))
                    {
                        result.RemoveAt(i);
                        i--;
                    }
                }
            }
            return result;
        }

        /// <summary>
        /// detect if there is a barrier between the given two points
        /// </summary>
        /// <param name="start"></param>
        /// <param name="destination"></param>
        /// <returns></returns>
        internal static bool HasBarrier(Point start, Point destination)
        {
            //if the start and destination are identical point
            if (start == destination)
            {
                return false;
            }

            // the center position of this two pints
            var point1 = new Vector2(start.X + 0.5f, start.Y + 0.5f);
            var point2 = new Vector2(destination.X + 0.5f, destination.Y + 0.5f);

            // according to the vertical and horizontal distance between this two points to judge the loop direction
            var distX = Math.Abs(destination.X - start.X);
            var distY = Math.Abs(destination.Y - start.Y);

            /**true: horizontal，false: vertical*/
            var loopDirection = (distX > distY);

            /**the line function between start and destination*/
            var lineFuction = GetLineFunc(point1, point2);


            /** loop start value */
            int loopStart;

            /** loop end value */
            int loopEnd;

            if (loopDirection)
            {
                loopStart = Math.Min(start.X, destination.X);
                loopEnd = Math.Max(start.X, destination.X);

                // begin the loop
                for (var i = (float)loopStart; i <= (float)loopEnd; i++)
                {
                    // for the start and destination point we need the center point
                    // for other points we only need the topleft corner position
                    if (Math.Abs(i - loopStart) <= 0) i += 0.5f;
                    // according to the x get the y value
                    var yPos = lineFuction.X * i + lineFuction.Y;

                    //check if the there is a barrier in this line
                    var passedNodeList = GetNodesUnderPoint(i, yPos);
                    foreach (var passedNode in passedNodeList)
                    {
                        if (ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mStaticBarriers.ContainsKey(passedNode)
                            || ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mDynamicBarriers.ContainsKey(passedNode))//ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mBarriers[passedNode.X][passedNode.Y])
                        {
                            return true;
                        }
                    }
                    if (Math.Abs(i - (loopEnd + 0.5f)) <= 0) i -= 0.5f;
                }
            }
            else
            {
                loopStart = Math.Min(start.Y, destination.Y);
                loopEnd = Math.Max(start.Y, destination.Y);

                for (var i = (float)loopStart; i <= (float)loopEnd; i++)
                {
                    if (Math.Abs(i - loopStart) <= 0) i += 0.5f;
                    // according to the line function to get the y value
                    var xPos = (i - lineFuction.Y) / lineFuction.X;

                    var passedNodeList = GetNodesUnderPoint(xPos, i);

                    foreach (var passedNode in passedNodeList)
                    {
                        if (ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mStaticBarriers.ContainsKey(passedNode)
                            || ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mDynamicBarriers.ContainsKey(passedNode))
                        {
                            return true;
                        }
                    }

                    if (Math.Abs(i - (loopEnd + 0.5f)) <= 0) i -= 0.5f;
                }
            }
            return false;
        }

        public static Vector2 Normalize(Vector2 mv)
        {
            return new Vector2(mv.X / mv.Length(), mv.Y / mv.Length());
        }
    }
}
