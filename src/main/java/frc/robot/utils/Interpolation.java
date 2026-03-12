package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Arrays;
import java.util.Collections;

/**
 * A prepared set of values for linear interpolation. Expensive to create, cheap to reuse.
 * Christian was here.
 */
public class Interpolation {
    private final Double[] input;
    private final Double[] output;

    /**
     * Returns a read-only view of the sorted, deduplicated input values
     */
    public List<Double> getInput() {
        return Collections.unmodifiableList(Arrays.asList(input));
    }
    /**
     * Returns a read-only view of the output values, sorted by their corresponding inputs.
     */
    public List<Double> getOutput() {
        return Collections.unmodifiableList(Arrays.asList(output));
    }

    /**
     * Copies and prepares data for linear interpolation. If the source data is in Arrays, use `Arrays.asList(your_array)` to work with this function.
     * This will sort the pairs by their input value, making this operation slightly expensive. Also note the thrown exceptions
     * @throws IllegalArgumentException if the input and output lists aren't the same length, or if there is a duplicated input value
     */
    public Interpolation(List<Double> input, List<Double> output) {
        if (input.size() != output.size()) throw new IllegalArgumentException("Input and Output datasets for interpolation have different sizes! " + input.size() + " != " + output.size());
        ArrayList<Double> tmp_input = new ArrayList<Double>(input.size());
        ArrayList<Double> tmp_output = new ArrayList<Double>(input.size());
        for (int i = 0; i < input.size(); i++) {
            Double entry = input.get(i);
            int search_index = Collections.binarySearch(tmp_input, entry);
            if (search_index >= 0) throw new IllegalArgumentException("Duplicate input value " + entry + "! Mapping must be a true function.");
            search_index = -(search_index + 1);
            tmp_input.add(search_index, entry);
            tmp_output.add(search_index, output.get(i));
        }
        this.input = tmp_input.toArray(new Double[0]);
        this.output = tmp_output.toArray(new Double[0]);
    }


    /**
     * Returns the clamped result of interpolating the given input value
     */
    public double interpolate(double input) {
        int index = Arrays.binarySearch(this.input, input);
        if (index >= 0) return this.output[index];
        index = -(index + 1);
        //System.out.println(index);
        if (index == 0) return this.output[0];
        if (index == this.input.length) return this.output[this.output.length - 1];
        double left_input = this.input[index - 1];
        double left_output = this.output[index - 1];
        double right_input = this.input[index];
        double right_output = this.output[index];
        return (((input - left_input) / (right_input - left_input)) * (right_output - left_output)) + left_output;
    }
}
