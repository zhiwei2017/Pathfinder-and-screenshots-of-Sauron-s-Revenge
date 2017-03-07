using System;
using System.Collections.Generic;

namespace SauronsRevenge.Object_Handling.PathFinder
{
    public sealed class BinaryHeap<T>
    {
        private T[] mData;

        private int mSize;

        private Comparison<T> mComparison;

        public BinaryHeap(Comparison<T> comparison)
        {
            Constructor(4, comparison);
        }

        private void Constructor(int capacity, Comparison<T> comparison)
        {
            mData = new T[capacity];
            mComparison = comparison ?? Comparer<T>.Default.Compare;
        }

        public int Size => mSize;

        /// <summary>
        /// Add an item to the heap
        /// </summary>
        /// <param name="item"></param>
        public void Insert(T item)
        {
            if (mSize == mData.Length)
                Resize();
            mData[mSize] = item;
            HeapifyUp(mSize);
            mSize++;
        }

        /// <summary>
        /// Extract the item of the root
        /// </summary>
        /// <returns></returns>
        public T Pop()
        {
            T item = mData[0];
            mSize--;
            mData[0] = mData[mSize];
            HeapifyDown(0);
            return item;
        }

        private void Resize()
        {
            T[] resizedData = new T[mData.Length * 2];
            Array.Copy(mData, 0, resizedData, 0, mData.Length);
            mData = resizedData;
        }

        private void HeapifyUp(int childIdx)
        {
            if (childIdx > 0)
            {
                int parentIdx = (childIdx - 1) / 2;
                if (mComparison.Invoke(mData[childIdx], mData[parentIdx]) > 0)
                {
                    // swap parent and child
                    T t = mData[parentIdx];
                    mData[parentIdx] = mData[childIdx];
                    mData[childIdx] = t;
                    HeapifyUp(parentIdx);
                }
            }
        }

        private void HeapifyDown(int parentIdx)
        {
            int leftChildIdx = 2 * parentIdx + 1;
            int rightChildIdx = leftChildIdx + 1;
            int smallestChildIdx = parentIdx;
            if (leftChildIdx < mSize && mComparison.Invoke(mData[leftChildIdx], mData[smallestChildIdx]) > 0)
            {
                smallestChildIdx = leftChildIdx;
            }
            if (rightChildIdx < mSize && mComparison.Invoke(mData[rightChildIdx], mData[smallestChildIdx]) > 0)
            {
                smallestChildIdx = rightChildIdx;
            }
            if (smallestChildIdx != parentIdx)
            {
                T t = mData[parentIdx];
                mData[parentIdx] = mData[smallestChildIdx];
                mData[smallestChildIdx] = t;
                HeapifyDown(smallestChildIdx);
            }
        }
    }
}
