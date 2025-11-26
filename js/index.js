import { test3 } from "../pkg/index.js";

import("../pkg/index.js").catch(console.error);

// await greet();
const button = document.getElementById('btn-connect');
button.addEventListener('click', async function() {
await test3();
});
