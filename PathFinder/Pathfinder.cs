using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;

namespace SauronsRevenge.Object_Handling.PathFinder
{
    /// <summary>
    /// class for finding a path from Vector A to Vector B
    /// </summary>
    public abstract class Pathfinder
    {
        /// <summary>
        /// computes the length of the distance from two given vectors.
        /// </summary>
        /// <param name="startPosition">first Position</param>
        /// <param name="endPosition">second position</param>
        /// <returns>bee line distance from the both vectors</returns>
        protected static float LengthOfDistanceVector(Vector2 startPosition, Vector2 endPosition)
        {
            var distanceVector = startPosition - endPosition;
            var distanceX = distanceVector.X;
            var distanceY = distanceVector.Y;
            return (float)Math.Sqrt(distanceX * distanceX + distanceY * distanceY);
        }

        /// <summary>
        /// heuristic function for A*
        /// </summary>
        /// <param name="startPosition"></param>
        /// <param name="endPosition"></param>
        /// <returns></returns>
        private static int Heuristic(Point startPosition, Point endPosition)
        {
            return Math.Abs(startPosition.X - endPosition.X) + Math.Abs(startPosition.Y - endPosition.Y);
        }

        
        /// <summary>
        /// out put the path
        /// </summary>
        /// <param name="before"></param>
        /// <param name="after"></param>
        /// <param name="camefromBegin"></param>
        /// <param name="cameFromEnd"></param>
        /// <returns></returns>
        private static List<Point> PathOutPut(Point before, Point after, 
            IReadOnlyDictionary<Point, Point> camefromBegin, IReadOnlyDictionary<Point, Point> cameFromEnd)
        {
            var result = new List<Point> { before, after };

            var b = before;
            while (camefromBegin[b] != b)
            {
                b = camefromBegin[b];
                result.Insert(0, b);
            }
            var a = after;
            while (cameFromEnd[a] != a)
            {
                a = cameFromEnd[a];
                result.Add(a);
            }
            result.RemoveAt(0);
            Floyd(ref result);
            return result;
        }

        /// <summary>
        /// comparator for the f value comparison
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <returns></returns>
        private static int Compare(Tuple<Point, long> x, Tuple<Point, long> y)
        {
            if (x.Item2 < y.Item2)
            {
                return 1;
            }
            if (x.Item2 > y.Item2)
            {
                return -1;
            }
            return 0;
        }

        private static void FloydVector(ref Point target, Point n1, Point n2)
        {
            target.X = n1.X - n2.X;
            target.Y = n1.Y - n2.Y;
        }

        /// <summary>
        /// floyed line smooth func
        /// </summary>
        /// <param name="path"></param>
        /// <returns></returns>
        private static void Floyd(ref List<Point> path)
        {
            if (path == null) return;
            var index = 0;
            if (path.Count > 2)
            {
                while (index < path.Count - 2)
                {
                    if (Math.Abs(path[index + 2].X - path[index].X) == 1
                        && Math.Abs(path[index + 2].Y - path[index].Y) == 1)
                    {
                        path.RemoveAt(index + 1);
                    }
                    index++;
                }
            }
            var len = path.Count;
            if (len > 2)
            {
                var vector = new Point(0, 0);
                var tempVector = new Point(0, 0);
                // loop all the node in the path and unite all the points which are in a same row 
                FloydVector(ref vector, path[len - 1], path[len - 2]);
                for (var i = path.Count - 3; i >= 0; i--)
                {
                    FloydVector(ref tempVector, path[i + 1], path[i]);
                    if (vector.X == tempVector.X && vector.Y == tempVector.Y)
                    {
                        path.RemoveAt(i + 1);
                    }
                    else
                    {
                        vector.X = tempVector.X;
                        vector.Y = tempVector.Y;
                    }
                }
            }
            // delete the inflexions
            len = path.Count;
            for (var i = len - 1; i >= 0; i--)
            {
                for (var j = 0; j <= i - 2; j++)
                {
                    if (!MathUi.HasBarrier(path[i], path[j]))
                    {
                        for (var k = i - 1; k > j; k--)
                        {
                            path.RemoveAt(k);
                        }
                        i = j;
                        break;
                    }
                }
            }
        }

        /// <summary>
        /// find all the 4 direct neighbours
        /// </summary>
        /// <param name="start"></param>
        /// <returns></returns>
        private static List<Point> FindNeighbours(Point start)
        {
            var neighbours = new List<Point>();
            if ((start.X < 0 && start.Y < 0) || (start.X > 99 && start.Y > 99))
            {
                return neighbours;
            }
            if (start.X > 0) neighbours.Add(new Point(start.X - 1, start.Y));
            if (start.X <  99) neighbours.Add(new Point(start.X + 1, start.Y));
            if (start.Y > 0) neighbours.Add(new Point(start.X, start.Y - 1));
            if (start.Y < 99) neighbours.Add(new Point(start.X, start.Y + 1));
            return neighbours;
        }

        /// <summary>
        /// bidirectional A*
        /// </summary>
        /// <param name="start"></param>
        /// <param name="destination"></param>
        /// <returns></returns>
        private List<Point> BidirectionAStar(Point start, Point destination)
        {
            var openListBegin = new BinaryHeap<Tuple<Point, long>>(Compare);
            openListBegin.Insert(Tuple.Create(start, 0L));

            var openListEnd = new BinaryHeap<Tuple<Point, long>>(Compare);
            openListEnd.Insert(Tuple.Create(destination, 0L));

            var cameFromBegin = new Dictionary<Point, Point> { [start] = start };
            var costSoFarBegin = new Dictionary<Point, int> { [start] = 0 };

            var cameFromEnd = new Dictionary<Point, Point> { [destination] = destination };
            var costSoFarEnd = new Dictionary<Point, int> { [destination] = 0 };
            while (openListBegin.Size > 0 && openListEnd.Size > 0)
            {
                var currentBegin = openListBegin.Pop();

                foreach (var next in FindNeighbours(currentBegin.Item1))
                {
                    if (ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mStaticBarriers.ContainsKey(next)
                        || ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mDynamicBarriers.ContainsKey(next)
                            //ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mBarriers[next.X][next.Y] 
                        || cameFromBegin.ContainsKey(next))
                    {
                        continue;
                    }
                    if (cameFromEnd.ContainsKey(next))
                    {
                        return PathOutPut(currentBegin.Item1, next, cameFromBegin, cameFromEnd);
                    }
                    var newCost = costSoFarBegin[currentBegin.Item1] + 1;
                    if (costSoFarBegin.ContainsKey(next) && !(newCost < costSoFarBegin[next])) continue;
                    costSoFarBegin[next] = newCost;
                    long priority = newCost + Heuristic(next, destination);

                    openListBegin.Insert(Tuple.Create(next, priority));
                    
                    cameFromBegin[next] = currentBegin.Item1;
                }

                var currentEnd = openListEnd.Pop();

                foreach (var next in FindNeighbours(currentEnd.Item1))
                {
                    if (ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mStaticBarriers.ContainsKey(next)
                        || ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mDynamicBarriers.ContainsKey(next)
                            //ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mBarriers[next.X][next.Y] 
                        || cameFromEnd.ContainsKey(next))
                    {
                        continue;
                    }
                    if (cameFromBegin.ContainsKey(next))
                    {
                        return PathOutPut(next, currentEnd.Item1, cameFromBegin, cameFromEnd);
                    }
                    var newCost = costSoFarEnd[currentEnd.Item1] + 1;
                    if (costSoFarEnd.ContainsKey(next) && !(newCost < costSoFarEnd[next])) continue;
                    costSoFarEnd[next] = newCost;
                    long priority = newCost + Heuristic(next, start);
                    openListEnd.Insert(Tuple.Create(next, priority));
                    cameFromEnd[next] = currentEnd.Item1;
                }
            }
            return new List<Point>();
        }

        /// <summary>
        ///  find the path
        /// </summary>
        /// <returns></returns>
        protected List<Vector2> PathFinding(Vector2 start, Vector2 end)
        {                 
            var result = new List<Vector2>();                               
            var startPosX = Math.Round(start.X / 32f) > 99 ? 99 : Math.Round(start.X / 32f);
            var startPosY = Math.Round(start.Y / 32f) > 99 ? 99 : Math.Round(start.Y / 32f);
            var startPos = new Point((int) startPosX, (int) startPosY);

            var endPosX = Math.Round(end.X / 32) > 99 ? 99 : Math.Round(end.X / 32);
            var endPosY = Math.Round(end.Y / 32) > 99 ? 99 : Math.Round(end.Y / 32);
            var endPos = new Point((int) endPosX, (int) endPosY);
            if ((startPosX < 0 && startPosY < 0) || (startPosX > 99 && startPosY > 99)
                || (endPosX < 0 && endPosY < 0) || (endPosX > 99 && endPosY > 99))
            {
                return result;
            }
            if (ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mStaticBarriers.ContainsKey(endPos)
                || ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.mDynamicBarriers.ContainsKey(endPos))
            {
                endPos = ScreenManager.mPlayScreen.mObjectHandler.mAbstractMap.FindSubstitutePosition(endPos);
                if (endPos == Point.Zero)
                {
                    return result;
                }
            }
            var hasBarrier = MathUi.HasBarrier(startPos, endPos);
            if (hasBarrier)
            {
                var path = BidirectionAStar(startPos, endPos);
                result.AddRange(path.Select(node => new Vector2(node.X * 32, node.Y * 32)));
                
            }
            else
            {
                result.Add(new Vector2(endPos.X * 32, endPos.Y * 32));
            }
            return result;
        }
    }
}