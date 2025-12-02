/**
 * Text selection types
 */

export interface TextSelection {
  text: string;
  position: {
    x: number;
    y: number;
  };
}
