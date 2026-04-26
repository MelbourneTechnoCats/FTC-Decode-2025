// adapted from: https://stackoverflow.c om/a/4859279

package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

public class ArrayIndexComparator<T extends Comparable<? super T>> implements Comparator<Integer> {
    private final T[] m_array;

    public ArrayIndexComparator(T[] array) {
        m_array = array;
    }

    public Integer[] getIndices() {
        Integer[] idxs = new Integer[m_array.length];
        for (int i = 0; i < m_array.length; i++)
            idxs[i] = i;
        return idxs;
    }

    @Override
    public int compare(Integer a, Integer b) {
        return m_array[a].compareTo(m_array[b]);
    }

    public Integer[] getSortedIndices(boolean descending) {
        Integer[] idxs = getIndices();
        Arrays.sort(idxs, this);
        if (descending) Collections.reverse(Arrays.asList(idxs));
        return idxs;
    }

    public Integer[] getSortedIndices() {
        return getSortedIndices(false); // default to ascending
    }
}
